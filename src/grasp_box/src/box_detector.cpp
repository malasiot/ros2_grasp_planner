#include "box_detector.hpp"

#include <cvx/geometry/kdtree.hpp>
#include <cvx/math/rng.hpp>
#include <fstream>

using namespace std;
using namespace Eigen;
using namespace cvx;

using LabelList = std::vector<uint>;
using ColorMap = std::vector<cv::Vec3b>;

static RNG g_rng;

std::tuple<PointCloud, LabelList, ColorMap>
depthToPointCloud(const cv::Mat &depth, const cv::Mat &mask, const cv::Mat &seg, const Camera &cam, uint sample_step = 1)
{
    float center_x = cam.cx_;
    float center_y = cam.cy_;

    const double unit_scaling = 0.001;

    float constant_x = unit_scaling / cam.fx_;
    float constant_y = unit_scaling / cam.fy_;

    cv::Mat_<ushort> depthx(depth);
    cv::Mat_<uchar> maskx(mask);
    cv::Mat_<cv::Vec3b> segx(seg);

    std::unordered_map<uint, uint> clr_map;

    PointCloud coords;
    LabelList labels;
    ColorMap colors;

    uint n_labels = 0;

    for (int i = 0; i < depth.rows; i += sample_step)
    {
        for (int j = 0; j < depth.cols; j += sample_step)
        {
            ushort val = depthx[i][j];

            if (val == 0 || maskx[i][j] == 0)
                continue;

            Vector4f p((j - center_x) * val * constant_x,
                       (i - center_y) * val * constant_y,
                       val * unit_scaling, 1.f);
            Vector3f pt = (cam.tr_ * p).head(3);

            cv::Vec3b clr = segx[i][j];
            uint b = clr[0], g = clr[1], r = clr[2];

            uint hash = (r << 16) + (g << 8) + b, label;

            auto it = clr_map.find(hash);
            if (it == clr_map.end())
            {
                clr_map.emplace(hash, n_labels);
                label = n_labels++;
            }
            else
            {
                label = it->second;
            }

            coords.push_back(pt);
            labels.push_back(label);
            colors.push_back(clr);
        }
    }

    return {coords, labels, colors};
}

static void saveCloudToOBJ(const string &fpath, const PointCloud &cloud)
{
    ofstream strm(fpath);

    for (const auto &v : cloud)
    {
        strm << "v " << v.adjoint() << endl;
    }
}

constexpr float inf = std::numeric_limits<float>::infinity();
static inline bool is_valid(const Vector3f &n) { return std::isfinite(n.x()); }
static inline void set_invalid(Vector3f &v) { v.x() = inf; }

static PointCloud computeNormals(const PointCloud &pts, const LabelList &labels, KDTree3 &search, float radius, float sample_prob)
{

    PointCloud normals(pts.size());

#pragma omp parallel for
    for (uint k = 0; k < pts.size(); k++)
    {

        if (g_rng.uniform<float>(0, 1) > sample_prob)
        {
            set_invalid(normals[k]);
            continue;
        }

        const Vector3f &p = pts[k];

        // find supporting points

        vector<uint> support;
        search.withinRadius(p, radius, support);

        uint n = support.size();

        if (n < 3)
        {
            set_invalid(normals[k]);
            continue;
        }

        // collect points
        PointCloud cpts;

        std::map<uint, uint> histogram;

        for (uint i : support)
        {
            uint q = labels[i];
            histogram[q]++;
        }

        // find dominant cluster
        auto max_label = std::max_element(histogram.begin(), histogram.end(),
                                          [](const pair<uint, uint> &p1, const pair<uint, uint> &p2)
                                          { return p1.second < p2.second; })
                             ->first;

        for (uint i : support)
        {
            uint q = labels[i];
            if (q == max_label)
            {
                cpts.push_back(pts[i]);
            }
        }

        if (cpts.size() < 3)
        {
            set_invalid(normals[k]);
            continue;
        }

        Eigen::Map<Matrix<float, Dynamic, 3, RowMajor>> mat((float *)cpts.data(), cpts.size(), 3);
        VectorXf centroid = mat.colwise().mean();
        MatrixXf centered = mat.rowwise() - centroid.adjoint();
        MatrixXf cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
        JacobiSVD<Matrix3f> svd(cov, ComputeFullU);
        Vector3f normal = svd.matrixU().col(2);

        normals[k] = normal;
    }

    return normals;
}

static void flipNormalsTowardsViewpoint(const PointCloud &pts, PointCloud &normals, const Vector3f &vp)
{
    for (uint i = 0; i < pts.size(); i++)
    {
        const auto &p = pts[i];
        auto &n = normals[i];
        if (is_valid(n))
        {
            float t = (vp - p).dot(n);
            if (t < 0)
                n = -n;
        }
    }
}

static void writeCloudToPLY(const std::string &filename,
                            const PointCloud &pts, const ColorMap &clrs)
{
    ofstream strm(filename);

    strm << "ply" << endl;
    strm << "format ascii 1.0" << endl;
    strm << "element vertex " << pts.size() << endl;
    strm << "property float x" << endl;
    strm << "property float y" << endl;
    strm << "property float z" << endl;
    strm << "property uchar red" << endl;
    strm << "property uchar green" << endl;
    strm << "property uchar blue" << endl;
    strm << "end_header" << endl;

    for (uint i = 0; i < pts.size(); i++)
    {
        const auto &p = pts[i];
        const auto &c = clrs[i];

        strm << p.adjoint() << ' ' << (int)c[2] << ' ' << (int)c[1] << ' ' << (int)c[0] << endl;
    }
}

struct Mesh
{
    std::vector<Vector3f> vertices_, normals_;
    std::vector<uint> vtx_indices_, normal_indices_;
};

void createMeshFromBox(const Box &box, Mesh &mesh)
{

    Affine3f tr = Translation3f(box.center_) * AngleAxisf(box.theta_, Vector3f::UnitZ());

    double sx = box.sz_.x() / 2.0;
    double sy = box.sz_.y() / 2.0;
    double sz = box.sz_.z() / 2.0;

    mesh.normals_ = {{0.0, 0.0, 1.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {-1.0, 0.0, 0.0}};
    mesh.vertices_ = {{-sx, +sy, +sz}, {+sx, +sy, +sz}, {+sx, -sy, +sz}, {-sx, -sy, +sz}, {-sx, +sy, -sz}, {+sx, +sy, -sz}, {+sx, -sy, -sz}, {-sx, -sy, -sz}};

    mesh.vtx_indices_ = {1, 0, 3, 7, 4, 5, 4, 0, 1, 5, 1, 2, 2, 3, 7, 0, 4, 7, 1, 3, 2, 7, 5, 6, 4, 1, 5, 5, 2, 6, 2, 7, 6, 0, 7, 3};
    mesh.normal_indices_ = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5, 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 5};

    for (auto &p : mesh.vertices_)
    {
        p = (tr * p.homogeneous()).head(3);
    }
}

void saveMeshOBJ(ostream &strm, const Mesh &mesh, uint &offset)
{
    for (const auto &v : mesh.vertices_)
    {
        strm << "v " << v.adjoint() << endl;
    }

    for (const auto &n : mesh.normals_)
    {
        strm << "vn " << n.adjoint() << endl;
    }

    for (size_t i = 0; i < mesh.vtx_indices_.size(); i += 3)
    {
        uint v1 = mesh.vtx_indices_[i] + offset * 8 + 1;
        uint v2 = mesh.vtx_indices_[i + 1] + offset * 8 + 1;
        uint v3 = mesh.vtx_indices_[i + 2] + offset * 8 + 1;
        uint n1 = mesh.normal_indices_[i] + offset * 6 + 1;
        uint n2 = mesh.normal_indices_[i + 1] + offset * 6 + 1;
        uint n3 = mesh.normal_indices_[i + 2] + offset * 6 + 1;
        strm << "f " << v1 << "//" << n1 << ' ';
        strm << v2 << "//" << n2 << ' ';
        strm << v3 << "//" << n3 << endl;
    }

    offset++;
}

void keepUpFacingNormals(PointCloud &normals, float thresh)
{
    for (auto &n : normals)
    {
        if (is_valid(n))
        {
            if (fabs(1 - n.z()) > thresh)
            {
                set_invalid(n);
            }
        }
    }
}

vector<Box> findBoxClusters(const PointCloud &pts, const PointCloud &normals, const LabelList &labels, std::map<uint, BoxCluster> &clusters, float min_cluster_size, float depth_thresh = 0.01)
{
    // clusters have the same label and a subset of points are pointing upwards

    std::map<uint, PointCloud> cluster_pts;
    std::map<uint, float> cluster_z;

    for (uint i = 0; i < pts.size(); i++)
    {
        const auto &p = pts[i];
        const auto &n = normals[i];
        const auto &label = labels[i];

        if (is_valid(n))
        {

            auto it = cluster_pts.find(label);
            if (it == cluster_pts.end())
            {
                PointCloud pcl;
                pcl.push_back(p);
                cluster_pts.emplace(label, std::move(pcl));
            }
            else
            {
                PointCloud &c = it->second;
                c.emplace_back(p);
            }
        }
    }
    // compute average z of collected clusters
    for (const auto &cp : cluster_pts) {

        float z = 0.f;
        for (const auto &p : cp.second)
        {
            z += p.z();
        }
        z /= cp.second.size();

        cluster_z[cp.first] = z;
    }

    // iterate over cloud points and collect points for each cluster that have z close to the average of upfacing points
    for (uint i = 0; i < pts.size(); i++)
    {
        const auto &p = pts[i];
        const auto &label = labels[i];

        auto it = cluster_z.find(label);
        if (it == cluster_z.end())
            continue;

        float z = it->second;

        if (fabs(p.z() - z) < depth_thresh)
        {
            auto it = clusters.find(label);
            if (it == clusters.end())
            {
                BoxCluster box;
                box.coords_.emplace_back(p);
                clusters.emplace(label, std::move(box));
            }
            else
            {
                BoxCluster &box = it->second;
                box.coords_.emplace_back(p);
            }
        }
    }

   
    vector<Box> boxes ;
    for (auto &cp : clusters)
    {
        BoxCluster &cluster = cp.second;

        if ( cluster.coords_.size() < min_cluster_size ) continue ;

        float maxz = std::max_element(cluster.coords_.begin(), cluster.coords_.end(), [](const Vector3f &p1, const Vector3f &p2)
                                      { return p1.z() < p2.z(); })
                         ->z();
        float minz = -0.05;

        float height = maxz - minz;

        vector<cv::Point2f> pj_pts;
        for (const auto &p : cluster.coords_)
            pj_pts.emplace_back(p.x(), p.y());
        cv::RotatedRect rect = cv::minAreaRect(pj_pts);

        if ( rect.angle > 45 ) {
            rect.angle = 90 - rect.angle ;
            rect.size = cv::Size2f(rect.size.height, rect.size.width) ;
        }
        
        Box box ;
        box.theta_ = rect.angle * M_PI / 180.0;
        cout << rect.angle << ' ' << rect.size.width << ' ' << rect.size.height << endl ;

        box.center_ = Vector3f{rect.center.x, rect.center.y, minz + height / 2.0};
        box.sz_ = Vector3f{rect.size.width, rect.size.height, height};

        boxes.emplace_back(box) ;
    }

    return boxes ;
}

static void saveBoxes(const string &fpath, const vector<Box> &boxes) {
    ofstream strm(fpath);
    uint offset = 0;

    for(const auto &box: boxes ) {
        Mesh mesh;
        createMeshFromBox(box, mesh);

        saveMeshOBJ(strm, mesh, offset);
    }
}
std::vector<Box> BoxDetector::detect(const Camera &cam, const cv::Mat &depth, const cv::Mat &mask, const cv::Mat &seg_mask)
{
    auto [pcl, labels, colors] = depthToPointCloud(depth, mask, seg_mask, cam, 2);

#ifdef DEBUG
    writeCloudToPLY("cloud.ply", pcl, colors);
#endif

    KDTree3 search(pcl);
    auto normals = computeNormals(pcl, labels, search, params_.nrm_radius_, params_.nrm_sampling_prob_);

    Vector3f vp = (cam.tr_.inverse() * Vector4f{0, 0, 0, 1}).head(3);

    flipNormalsTowardsViewpoint(pcl, normals, vp);

    keepUpFacingNormals(normals, params_.up_facing_thresh_);

    std::map<uint, BoxCluster> clusters;
    
    auto boxes = findBoxClusters(pcl, normals, labels, clusters, params_.min_cluster_sz_ * pcl.size(), params_.depth_thresh_);

#ifdef DEBUG
    saveBoxes("boxes.obj", boxes);
#endif

    return boxes ;
}
