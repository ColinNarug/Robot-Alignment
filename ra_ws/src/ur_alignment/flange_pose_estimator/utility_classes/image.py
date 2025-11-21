import cv2
import numpy as np

from .parameters_handling import Params 
from .drawer import Drawer, DrawProxy


class Image:

    def __init__(self, frame: np.ndarray, params: Params, undistort: bool = False) -> None:
         
        # Parameters
        self.params = params
        self.camera_params = self.params.camera_params
        self.flange_model_params = self.params.flange_model_params
        
        # Frame
        self.set_base_frame(frame, undistort)
        
        # Extracted holes contours (ellipses)
        self.ellipses = [] 

        # Drawings
        self.drawer = Drawer(self.frame.copy(), self.params)
        self.draw = DrawProxy(self.drawer, self) 



    @property
    def num_ellipses(self) -> int:
        """
        Return the number of ellipses detected in the image.
        """
        return len(self.ellipses)



    def get_contour_extraction_status(self) -> bool:
        """
        Check if the number of detected ellipses is sufficient for the model.
        """

        success = len(self.ellipses) >= self.flange_model_params.MINIMUM_NUM_ACTIVE_HOLES

        if self.params.debug_params.DETAILS:
            if not success:
                print(f"-[DETAILS] Fit {len(self.ellipses)} ellipses, not enough (< {self.flange_model_params.MINIMUM_NUM_ACTIVE_HOLES}).")
            else:
                print(f"-[DETAILS] Fit {len(self.ellipses)} ellipses.")

        if success and self.params.debug_params.DRAW_RESULTS: self.draw.ellipses(self.ellipses)
        
        return success


    def set_base_frame(self, frame: np.ndarray, undistort: bool = False) -> None:
        """
        Set the image frame, optionally applying undistortion based on camera parameters.
        """
        if undistort:

            if self.camera_params.CAMERA_MATRIX is not None and self.camera_params.DISTORTION_COEFFS is not None:
                self.frame = cv2.undistort(frame, self.camera_params.CAMERA_MATRIX, self.camera_params.DISTORTION_COEFFS)
                self.images_with_drawings = [self.frame]
            else:
                raise ValueError(f"Cannot undistort: camera_matrix is {self.camera_params.CAMERA_MATRIX} and distortion_coeffs is {self.camera_params.DISTORTION_COEFFS}")
        
        else:
            self.frame = frame
            self.images_with_drawings = [self.frame]
            
