import numpy as np
import cv2
from typing import Tuple, Optional
import segmentation_models_pytorch as smp
import torch
from torch import nn
import torch.nn.functional as F

from ur_alignment.flange_pose_estimator.utility_classes import Ellipse
 

class EllipseExtractor:
    def __init__(self, params: object, timer: object) -> None:
        """
        Initialize fitter, read config, set device, load SMP model.
        """
        self.timer = timer
        self.params = params

        # Device used for inference
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Load segmentation model
        self.model: nn.Module = self._load_model(self.params.ellipse_extraction_params.MODEL_PATH)
  


    def _load_model(self, model_path: str) -> nn.Module:
        """
        Build SMP model with the SAME arch/encoder used in training and load weights.
        """

        model = smp.Unet(
            encoder_name    = self.params.ellipse_extraction_params.ENCODER_NAME,
            encoder_weights = None,
            in_channels     = self.params.ellipse_extraction_params.NUM_IN_CHANNELS,
            classes         = 1,                    # only holes
            activation=None,         
        ).to(self.device)

        # Load state_dict saved at training time (supports DataParallel prefixes).
        state = torch.load(model_path, map_location=self.device, weights_only=False)
        if isinstance(state, dict) and "state_dict" in state:
            state = state["state_dict"]
        state = {k.replace("module.", ""): v for k, v in state.items()}
        model.load_state_dict(state, strict=True)
        model.eval()
        return model



    def run_inference(self, ROI: np.ndarray) -> np.ndarray:
        """
        Run UNet inference on the ROI and return a binary mask (H,W) in {0,1}.
        """
        H, W = ROI.shape[:2]
        ROI = cv2.resize(ROI, self.params.ellipse_extraction_params.ROI_SIZE)  

        x = self._preprocess_roi(ROI)
        with torch.no_grad(): logits = self.model(x)  # inference
        mask01 = self._postprocess_mask(logits, (H, W))

        mask01 = cv2.resize(mask01, (H, W))  
        return mask01



    def mask_to_edge(self, mask01: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Extract a thin edge from the mask, keep ONLY the largest connected component,
        subsample the edge points, and cache them for later use.
        """

        # --- Morphological gradient to extract borders ---
        # The morphological gradient is defined as dilate(mask) - erode(mask).
        # This operation highlights the boundary pixels of the mask, producing a thin contour around the segmented region.
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.morphologyEx(mask01.astype(np.uint8), cv2.MORPH_GRADIENT, kernel)

        # --- Keep only the largest connected component ---
        # Select only the largest connected component (CC) based on area.
        num, labels, stats, _ = cv2.connectedComponentsWithStats(edges, connectivity=8)
        if num > 1:
            # Index 0 is the background, so we skip it.
            largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
            edges = (labels == largest).astype(np.uint8)

        # --- Extract coordinates of edge pixels ---
        # The nonzero() function gives us the row (y) and column (x) indices of all active pixels belonging to the final edge.
        ys, xs = np.nonzero(edges)

        # --- Cache the edge points ---
        # Store the (x, y) coordinates as float32 
        # If no points are found, cache an empty array.
        last_edge_points = (
            np.stack([xs.astype(np.float32), ys.astype(np.float32)], axis=1)
            if xs.size > 0 else np.empty((0, 2), dtype=np.float32)
        )

        return edges, last_edge_points


    def edge_to_ellipse(self, last_edge_points: np.ndarray, ID: int) -> tuple[bool, Optional[Ellipse]]:
        """
        Fit an ellipse from cached edge points .
        """
        points = last_edge_points if last_edge_points is not None else np.empty((0, 2), np.float32)
        if points.shape[0] < 5:
            if self.params.debug_params.DETAILS:
                print("-[ATTENTION] Not enough points in the ROI to fit an ellipse.")
            return False, None

        ellipse = Ellipse()
        ellipse.ID = ID
        ellipse.set_coefficients_from_points(points)  
        self.last_ellipse_roi = ellipse
        return True, ellipse


    @staticmethod
    def ellipse_ROI_to_full_image(ellipse_roi: Ellipse, ROI_coordinates: tuple[int, int, int, int]) -> Ellipse:
        """
        Map an ellipse from ROI space to full-image space.
        """
        x1, y1, _, _ = ROI_coordinates
        cx, cy, angle, a, b = ellipse_roi.get_parameters()

        ellipse_full_img = Ellipse()
        ellipse_full_img.ID = ellipse_roi.ID
        ellipse_full_img.set_parameters(cx + x1, cy + y1, angle, a, b)
        return ellipse_full_img


    def _preprocess_roi(self, ROI: np.ndarray) -> tuple[torch.Tensor, Tuple[int, int]]:
        """
        ConverT A ROI 256x256 BGR TO tensore (1,1,H,W) float32 in [0,1].
        """
        
        if self.params.ellipse_extraction_params.NUM_IN_CHANNELS == 1:
            gray = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY).astype(np.float32) / 255.0  # (H,W)
            ten = torch.from_numpy(gray).unsqueeze(0).unsqueeze(0).to(self.device)   # (1,1,H,W)
        else:
            img = cv2.cvtColor(ROI, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0    # (H,W,3)
            ten = torch.from_numpy(img).permute(2,0,1).unsqueeze(0).to(self.device)  # (1,3,H,W)
        return ten


    def _postprocess_mask(self, logits: torch.Tensor, orig_hw: Tuple[int, int]) -> np.ndarray:
        """
        Apply sigmoid + threshold, then resize back to original ROI size (256x256); return {0,1}.
        """
        # Convert logits -> probability -> binary mask
        prob = torch.sigmoid(logits)                               
        mask = (prob > self.params.ellipse_extraction_params.MIN_CONFIDENCE_HOLE).float()           
        mask = mask[0, 0:1, :, :]                                   

        # Resize back if necessary (should already be 256x256, but kept for safety)
        H0, W0 = orig_hw
        h, w = mask.shape[-2:]
        if (h, w) != (H0, W0):
            mask = F.interpolate(mask.unsqueeze(0), size=(H0, W0), mode="nearest").squeeze(0)

        # Convert to numpy {0,1}
        return mask[0].detach().cpu().numpy().astype(np.uint8)


    def get_ellipse(self,
                    ROI: np.ndarray,
                    ROI_coordinates: tuple[int, int, int, int],
                    ID: int
                    ) -> tuple[bool, Optional[Ellipse]]:
        """
        Main entry: run inference on ROI → fit ellipse → return in full-image coords.
        """

        mask_binary = self.run_inference(ROI)
        edges, last_edge_points = self.mask_to_edge(mask_binary)
        success, ellipse_roi = self.edge_to_ellipse(last_edge_points, ID)
        
        if not success: return False, None

        ellipse_full_img = self.ellipse_ROI_to_full_image(ellipse_roi, ROI_coordinates)
        self.last_ellipse_full = ellipse_full_img
                
        return True, ellipse_full_img

