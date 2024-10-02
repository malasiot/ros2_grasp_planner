import numpy as np
from pycocotools import mask
import pickle
import os


def combine_masks_points(result, cv_image, min_mask):
    """
    Combine masks into a single segmented image and save the results of the sam model, utillizing the points implementation as input prompt
    masks: compressed rle format
    scores: floats

    """
    segmented_image = np.zeros_like(cv_image, dtype=np.uint8)
    save_dict = {"rle_mask": [], "score": []}
    idx = 0
    for mask_item in result:
        rle = mask_item['segmentation']
        binary_mask = mask.decode(rle)
        if np.sum(binary_mask) > min_mask:
            idx += 1
            save_dict["rle_mask"].append(rle)
            save_dict["score"].append(mask_item["predicted_iou"])
            color = np.array(np.random.rand(3) * 255, np.uint8)  # Generate a random color
            ind = binary_mask > 0
            segmented_image[ind, :] = color

    return segmented_image


def combine_masks_boxes(masks, scores, cv_image, frame_id, min_mask):
    """
    Combine masks into a single segmented image and save the results of the sam model, utillizing boxes as input prompt
    masks: compressed rle format
    scores: floats

    """
    segmented_image = np.zeros_like(cv_image, dtype=np.uint8)
    save_dict = {"rle_mask": [], "score": []}
    idx = 0
    for ind, mask_item in enumerate(masks):
        rle = binary_mask_to_rle(mask_item)
        # binary_mask = mask.decode(rle)
        if np.sum(mask_item) > min_mask:
            idx += 1
            save_dict["rle_mask"].append(rle)
            save_dict["score"].append(scores[ind])
            color = np.array(np.random.rand(3) * 255, np.uint8)  # Generate a random color
            ind = mask_item > 0
            segmented_image[ind, :] = color

    path = './segmented_images'
    if not os.path.exists(path):
        os.makedirs(path)
    file_path = frame_id.replace(".jpg", "")

    with open(f"{path}/{file_path}.pkl", 'wb') as f:
        pickle.dump(save_dict, f)

    return segmented_image


def binary_mask_to_rle(binary_mask):
    """
    Convert binary mask to compressed RLE format.

    Args:
        binary_mask: 2D numpy array representing the binary mask.

    Returns:
        dict: Dictionary containing RLE encoded mask.
    """
    encoded_mask = mask.encode(np.array(binary_mask, order='F', dtype=np.uint8))
    encoded_mask['counts'] = encoded_mask['counts'].decode('utf-8')
    return encoded_mask
