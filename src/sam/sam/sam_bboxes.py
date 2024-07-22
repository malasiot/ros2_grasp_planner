from pathlib import Path
import torch

from segment_anything import sam_model_registry, \
    SamPredictor

# Free up GPU memory before loading the model
import gc


class SAMbboxes():
    def __init__(self, model_type, cuda_device=None) -> None:
        model_dir = Path(f"./model")
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

        self.predictor = SamPredictor(sam_model)

    def __del__(self):
        gc.collect()
        torch.cuda.empty_cache()

    def segment(self, img, boxes, multimask=False):
        self.predictor.set_image(img)
        return self.predictor.predict(
            box=boxes,
            multimask_output=multimask
        )
