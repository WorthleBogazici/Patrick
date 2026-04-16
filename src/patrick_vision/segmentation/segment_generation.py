import numpy as np
import torch
from ultralytics import FastSAM


class FastSAMMaskExtractor:
    def __init__(
        self,
        model_path="/home/ayca/sp_ws/src/3DObjectGeneration/models/FastSAM-x.pt",
        device=None,
        imgsz=1024,
        conf=0.6,
        iou=0.9,
        retina_masks=True
    ):
        self.device = device if device is not None else ("cuda" if torch.cuda.is_available() else "cpu")
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        self.retina_masks = retina_masks

        self.model = FastSAM(model_path)

    def extract_masks(self, frame_rgb: np.ndarray) -> np.ndarray:
  
        if frame_rgb is None:
            raise ValueError("frame_rgb is None.")

        if not isinstance(frame_rgb, np.ndarray):
            raise TypeError("frame_rgb must be a numpy array.")

        if frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
            raise ValueError(f"Expected frame_rgb shape (H, W, 3), got {frame_rgb.shape}.")

        if frame_rgb.dtype != np.uint8:
            frame_rgb = frame_rgb.astype(np.uint8)

        results = self.model(
            frame_rgb,
            device=self.device,
            retina_masks=self.retina_masks,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            verbose=False
        )

        h, w = frame_rgb.shape[:2]

        if len(results) == 0 or results[0].masks is None or results[0].masks.data is None:
            return np.empty((0, h, w), dtype=np.uint8)

        masks = results[0].masks.data  # torch tensor: [N, H, W]

        if masks.shape[0] == 0:
            return np.empty((0, h, w), dtype=np.uint8)

        masks_np = (masks > 0).to(torch.uint8).cpu().numpy()
        return masks_np
        
_extractor = None

def initialize_model(
    model_path="/home/ayca/sp_ws/src/3DObjectGeneration/models/FastSAM-x.pt",
    device=None,
    imgsz=1024,
    conf=0.6,
    iou=0.9,
    retina_masks=True
):
    global _extractor
    _extractor = FastSAMMaskExtractor(
        model_path=model_path,
        device=device,
        imgsz=imgsz,
        conf=conf,
        iou=iou,
        retina_masks=retina_masks
    )


def get_masks_from_rgb(frame_rgb: np.ndarray) -> np.ndarray:

    global _extractor

    if _extractor is None:
        initialize_model()

    return _extractor.extract_masks(frame_rgb)

