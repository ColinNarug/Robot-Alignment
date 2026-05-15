from spatialmath import SE3
import numpy as np
from typing import Optional, Tuple, List
from scipy.spatial.transform import Rotation as R

from .holes_edge_extraction import *
from .utility_classes import parse_config, Image, Timer, Params
from .pose_computation import PoseEstimator



class FlangePoseEstimator():

    def __init__(self, config: dict) -> None:
        """
        Initialize the pose estimation pipeline with the given YAML config.
        """

        # Extract parameters from config.YAML
        self.params: Params = parse_config(config)
        
        # Estimated poss of the last frame
        self.T_past: SE3  = None
        self.T_init: SE3  = None
        self.success: bool = False 
        self._init_pose()

        # Init the objects  
        self.timer:             Timer            = Timer()
        self.detector:          Detector         = Detector(self.params, self.timer)
        self.image_processor:   ImageProcessor   = ImageProcessor(self.params, self.timer)
        self.ellipse_fitter:    EllipseFitter    = EllipseFitter(self.params, self.timer)
        self.ellipse_extractor: EllipseExtractor = EllipseExtractor(self.params, self.timer)
        self.pose_estimator:    PoseEstimator    = PoseEstimator(self.params, self.timer, self.T_init)
        



    def _init_pose(self) -> None:
        """
        Set initial pose, for initialization
        """
        # init translation [m] and rotation vector [rad]
        # such that the flange is in front of the camera
        t = np.array([0,     0, 0]).reshape(3)
        r = np.array([0, np.pi, 0]).reshape(3)

        try:
            # Convert rotation vector (axis–angle) --> rotation matrix 
            Rm = R.from_rotvec(r).as_matrix()

            # Build SE3 with R and t, then set as initial pose
            self.T_init = SE3.Rt(Rm, t)
            self.T_past = self.T_init
            self.T      = self.T_init

        # Do not block startup if YAML is malformed
        except Exception: return



    def set_initial_pose(self, g: np.ndarray) -> None:
        """
        Initialize pose using the camera gravity vector.
        - y-axis is aligned with gravity direction (unit).
        - z-axis points into the camera.
        - x-axis is computed to complete a right-handed, orthonormal basis.
        The resulting pose T is an SE3 with zero translation.
        """
        # ---- checks ----
        if not isinstance(g, np.ndarray):
            raise TypeError("g must be a np.ndarray.")
        if g.size != 3:
            raise ValueError("g must be a 3-element vector.")
        n = np.linalg.norm(g)
        if n < 1e-12:
            raise ValueError("Gravity vector has near-zero norm.")

        # ---- normalize gravity -> y axis ----
        y = (g / n).astype(float)

        # ---- desired 'into-camera' z axis ----
        # Convention: z points into the camera (e.g., [0, 0, -1]).
        z_desired = np.array([0.0, 0.0, -1.0], dtype=float)

        # If y is almost parallel to z_desired, choose an alternative z to avoid degeneracy.
        if abs(float(np.dot(y, z_desired))) > 0.999:
            z_desired = np.array([1.0, 0.0, 0.0], dtype=float)

        # ---- orthogonalize z relative to y (Gram-Schmidt) ----
        # Make z perpendicular to y while staying close to desired
        z = z_desired - np.dot(z_desired, y) * y
        nz = np.linalg.norm(z)
        if nz < 1e-12:
            # Fallback (shouldn't happen, but guard anyway)
            z = np.array([0.0, 1.0, 0.0], dtype=float) - np.dot(np.array([0.0, 1.0, 0.0]), y) * y
            nz = np.linalg.norm(z)
            if nz < 1e-12:
                raise RuntimeError("Failed to build a valid z axis.")
        z /= nz

        # ---- compute x to make a right-handed frame ----
        x = np.cross(y, z)
        nx = np.linalg.norm(x)
        if nx < 1e-12:
            raise RuntimeError("Failed to build a valid x axis (degenerate basis).")
        x /= nx

        # Recompute z = x × y to ensure exact orthonormality and right-handedness
        z = np.cross(x, y)
        z /= np.linalg.norm(z)

        # ---- rotation matrix (columns are the axes of the new frame) ----
        Rm = np.column_stack((x, y, z))  # shape (3,3)

        # ---- build SE3 (zero translation) ----
        T = SE3.Rt(Rm, np.zeros(3))

        # ---- update estimator state ----
        # If your class expects creating/refreshing an internal PoseEstimator with T:
        self.pose_estimator = PoseEstimator(self.params, self.timer, T)
        self.T_past = T
        self.T = T

        

    def get_image_with_RF(self) -> np.ndarray:
        """
        Return the image with reference frame overlay, if available.
        """
        # note: "image.images_with_drawings" collected the following images:
        # 0 --> base
        # 1 --> base + ellipses
        # 2 --> base + ellipses + projected holes centers 
        # 3 --> base + ellipses + projected holes centers + pose with reference frame 
        
        if not self.success or not self.params.debug_params.DRAW_RESULTS:
            if len(self.image.images_with_drawings) > 1:
                return self.image.images_with_drawings[1]
            else:
                return self.image.images_with_drawings[0]

        self.image = self.pose_estimator.get_image()
        flange_with_RF = self.image.images_with_drawings[3]
        return flange_with_RF
    

    
    def get_homography_analysis_results(self) -> List[float]:
        """
        Return distances for homography matching analysis.
        """
        matching_distances = self.pose_estimator.get_homography_analysis_results()
        return matching_distances

    

    def init_pose_estimation(self) -> None:
        """
        Reset pose estimation state for a new frame.
        """
        self.T = None
        self.success = False
        
    

    def get_pose(self, frame: np.ndarray) -> Tuple[bool, Optional[SE3]]:
        """
        Processes a frame to estimate the pose of a flange.

        Returns:
            success (bool) -> Whether pose estimation was successful
            T (SE3 | None) -> Estimated flange pose relative to the camera frame
        """

        self.init_pose_estimation()
        self.last_failure_reason = "unknown"
        self.last_holes_detected = 0
        self.last_ellipses = 0

        # --------------------------------
        # PICTURE SETUP
        # --------------------------------
        try:
            self.image = Image(frame, self.params, undistort=True)
        except Exception as exc:
            self.last_failure_reason = f"image setup/undistort failed: {exc}"
            return False, None

        # --------------------------------
        # HOLES DETECTION
        # --------------------------------
        try:
            self.success, holes_detected = self.detector.detect(self.image.frame)
        except Exception as exc:
            self.last_failure_reason = f"holes detection exception: {exc}"
            return False, None

        try:
            self.last_holes_detected = len(holes_detected) if holes_detected is not None else 0
        except Exception:
            self.last_holes_detected = -1

        if not self.success or holes_detected is None or self.last_holes_detected <= 0:
            self.last_failure_reason = (
                f"holes detection failed: success={self.success}, "
                f"holes_detected={self.last_holes_detected}"
            )
            return False, None

        # ------------------------------------------
        # HOLES ELLIPSE CONTOUR EXTRACTION
        # ------------------------------------------
        for hole_detected in holes_detected:
            try:
                if self.params.method_choice == 0:
                    ROI_canny = self.image_processor.get_edge(hole_detected.ROI)
                    self.success, best_ellipse = self.ellipse_fitter.get_ellipse(
                        ROI_canny,
                        hole_detected.ROI_coordinates,
                        hole_detected.ID
                    )
                else:
                    self.success, best_ellipse = self.ellipse_extractor.get_ellipse(
                        hole_detected.ROI,
                        hole_detected.ROI_coordinates,
                        hole_detected.ID
                    )

                if self.success:
                    self.image.ellipses.append(best_ellipse)

            except Exception as exc:
                self.last_failure_reason = (
                    f"ellipse extraction exception on hole ID "
                    f"{getattr(hole_detected, 'ID', 'unknown')}: {exc}"
                )
                return False, None

        self.last_ellipses = len(self.image.ellipses)

        self.success = self.image.get_contour_extraction_status()
        if not self.success:
            self.last_failure_reason = (
                f"ellipse contour extraction failed: "
                f"holes_detected={self.last_holes_detected}, "
                f"ellipses={self.last_ellipses}, "
                f"method_choice={self.params.method_choice}"
            )
            return False, None

        # ------------------------------------------
        # POSE EXTRACTION FROM ELLIPSE
        # ------------------------------------------
        try:
            self.success, self.T = self.pose_estimator.get_pose(self.image)
        except Exception as exc:
            self.last_failure_reason = (
                f"pose extraction exception: "
                f"holes_detected={self.last_holes_detected}, "
                f"ellipses={self.last_ellipses}, "
                f"error={exc}"
            )
            return False, None

        if not self.success or self.T is None:
            self.last_failure_reason = (
                f"pose extraction failed: "
                f"holes_detected={self.last_holes_detected}, "
                f"ellipses={self.last_ellipses}"
            )
            return False, None

        self.last_failure_reason = ""
        return True, self.T

