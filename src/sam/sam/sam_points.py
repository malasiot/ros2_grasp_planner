from pathlib import Path
import torch
from ament_index_python.packages import get_package_share_directory

from segment_anything import sam_model_registry, \
    SamPredictor, SamAutomaticMaskGenerator

import yaml

# Free up GPU memory before loading the model
import gc


class SAMpoints():
    def __init__(self, params, model_type, cuda_device=None) -> None:
        """ initialize sam using the config's parameters """
        model_dir = Path("/app/model/")
        checkpoints = list(model_dir.glob(f'sam_{model_type}_*.pth'))
        
        if len(checkpoints) == 0:
            raise RuntimeError(f'No matching checkpoints for SAM model "{model_type}" was found in "{model_dir}"')

        if len(checkpoints) > 1:
            raise RuntimeError(f'No unique checkpoint for SAM model "{model_type}" found in "{model_dir}"')
      
        gc.collect()
        torch.cuda.empty_cache()

        sam_model = sam_model_registry[model_type](checkpoint=checkpoints[0])
        self.device = cuda_device

        if self.device is not None:
            sam_model.to(device=self.device)

        self.output_mode = "coco_rle"
        self.generator = SamAutomaticMaskGenerator(sam_model, output_mode=self.output_mode, **params)

    def __del__(self):
        gc.collect()
        torch.cuda.empty_cache()

    def segment(self, img):
        """ segment the images
        output -> results: Dictionary including the segmentations in rle format
                  scores: Predicting the mask iou with the ground truth
        """
        return self.generator.generate(img)
