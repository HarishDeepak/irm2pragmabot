import numpy as np
import torch
import cv2
from groundingdino.util.inference import load_model, predict
from groundingdino.util.utils import get_phrases_from_posmap
from groundingdino.datasets import transforms as T
from segment_anything import sam_model_registry, SamPredictor
from PIL import Image as PILImage

GDINO_CONFIG  = "/tmp/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
GDINO_WEIGHTS = "/catkin_ws/src/pragmabot/weights/groundingdino_swint_ogc.pth"
SAM_WEIGHTS   = "/catkin_ws/src/pragmabot/weights/sam_vit_h_4b8939.pth"
SAM_TYPE      = "vit_h"


class GroundedSAM:

    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.gdino = load_model(GDINO_CONFIG, GDINO_WEIGHTS, device=self.device)
        self.transform = T.Compose([
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        sam = sam_model_registry[SAM_TYPE](checkpoint=SAM_WEIGHTS)
        sam.to(self.device)
        self.predictor = SamPredictor(sam)

    def segment(
        self,
        image: np.ndarray,
        text: str,
        box_threshold: float = 0.3,
        text_threshold: float = 0.25,
    ) -> tuple:
        """
        Args:
            image: H×W×3 uint8 BGR (OpenCV / ros_numpy convention)
            text:  natural-language description of the target object
        Returns:
            mask       : H×W bool ndarray (True = object pixel)
            confidence : float (best GroundingDINO detection score, 0.0 if nothing found)
        """
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pil_img = PILImage.fromarray(rgb)
        img_tensor, _ = self.transform(pil_img, None)

        boxes, logits, _ = predict(
            model=self.gdino,
            image=img_tensor,
            caption=text,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            device=self.device,
        )

        if len(boxes) == 0:
            return np.zeros(image.shape[:2], dtype=bool), 0.0

        best_idx = int(logits.argmax())
        confidence = float(logits[best_idx])
        box = boxes[best_idx]  # normalised [cx, cy, w, h]

        H, W = image.shape[:2]
        cx, cy, bw, bh = box.tolist()
        x1 = int((cx - bw / 2) * W)
        y1 = int((cy - bh / 2) * H)
        x2 = int((cx + bw / 2) * W)
        y2 = int((cy + bh / 2) * H)

        self.predictor.set_image(rgb)
        masks, _, _ = self.predictor.predict(
            box=np.array([x1, y1, x2, y2]),
            multimask_output=False,
        )
        return masks[0].astype(bool), confidence