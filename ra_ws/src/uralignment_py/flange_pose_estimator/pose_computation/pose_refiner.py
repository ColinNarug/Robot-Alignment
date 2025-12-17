import numpy as np
import cv2
from spatialmath import SE3

from ur_alignment.flange_pose_estimator.utility_classes import Image, Hole
from .pose_computation_core import PoseEstimatorCore



class PoseRefiner(PoseEstimatorCore):

    def __init__(self, pose_estimator: PoseEstimatorCore) -> None:

        super().__init__(image = pose_estimator.image, 
                         T = pose_estimator.T,
                         T_past = pose_estimator.T_past, 
                         params = pose_estimator.params,
                         holes = pose_estimator.holes,
        )   
        self.success = False
        
    
    def get_image(self) -> Image:
        """
        Return the current Image object.
        """
        return self.image
    

    def get_holes(self) -> list[Hole]:
        """
        Return the current list of holes.
        """
        return self.holes
    

    def get_tracking_data(self) -> dict[int, np.ndarray] | None:
        """
        Return projected centers for each tracked hole to be used for future tracking.
        """
        if not self.success:
            return None

        R = self.T.R
        rvec, _ = cv2.Rodrigues(R)
        tvec = self.T.t

        holes_with_cam = [h for h in self.holes if h["camera"].center is not None]
        centers_holes_flangeRF = self.get_holes_centers(holes_with_cam, "flange", only_with_camera_repr=True)
        centers_proj, _ = cv2.projectPoints(centers_holes_flangeRF, rvec, tvec, self.params.camera_params.CAMERA_MATRIX, None)
        centers_proj = centers_proj.reshape(-1, 2)

        tracking_data: dict[int, np.ndarray] = {}
        for hole, pt in zip(holes_with_cam, centers_proj):
            tracking_data[int(hole.ID)] = pt

        return tracking_data


    def refine_holes(self) -> tuple[bool, list[Hole]]:
        """
        Perform some checks on the reconstructed holes and remove the ones not well-estimated
        """
        success, self.holes = self.refine_by_reprojection_error(self.holes)

        # add other refinements here...

        return success, self.holes
    


    def refine_by_reprojection_error(self, holes_not_filtered: list[Hole]) -> tuple[bool, list[Hole]]:
        """
        Project the 3-D hole centres (flange RF) on the image with the current
        (R, t). Compare each projection with the centre of the detected
        ellipse on the image. Keep only holes whose distance is below
        TRESHOLD_COARSE_REPROJECTION_ERROR.
        """
        pairs = [h for h in holes_not_filtered
                if h["camera"].center is not None and h.projection_edge is not None]

        if len(pairs) == 0:
            if self.debug_params.DETAILS:
                print("-[FAIL] No valid hole pairs (camera+projection) for reprojection check")
            return False, self._restore_holes_without_projection([], holes_not_filtered)

        objpoints = self.get_holes_centers(pairs, "flange")
        imgpoints = self.get_holes_projections_centers(pairs, only_with_camera_repr=True)

        CAMERA_MATRIX = self.camera_params.CAMERA_MATRIX
        distances = self.get_projection_distances_list(imgpoints, objpoints, self.T, CAMERA_MATRIX).reshape(-1)

        # >>> QUI era il bug: usavi self.holes[i] invece di pairs[i]
        kept = []
        for h, d in zip(pairs, distances):
            if self.debug_params.MORE_DETAILS:
                print(f"-[MORE DETAILS] Initial pose reprojection error = {float(d):.3f} [pix]")
            if d <= self.pose_estimation_params.TRESHOLD_COARSE_REPROJECTION_ERROR:
                kept.append(h)

        success = len(kept) >= self.flange_model_params.MINIMUM_NUM_ACTIVE_HOLES
        if not success and self.debug_params.DETAILS:
            print(f"-[FAIL] Initial reprojection error check failed: found {len(kept)} projections below threshold")

        kept = self._restore_holes_without_projection(kept, holes_not_filtered)
        return success, kept


    @staticmethod
    def project_3D_points_on_screen(points: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
        """
        Projects 3D points on the screen, described in the camera RF,
        and returns the corresponding 2D projection points.
        Accepts either a single 3D point or an array of 3D points.
        """

        # Convert input to at least 2D and float32
        points = np.asarray(points, dtype=np.float32)
        
        # If a single point is passed, reshape to (1, 3)
        if points.ndim == 1 and points.shape[0] == 3:
            points = points.reshape(1, 3)
        elif points.ndim == 2 and points.shape[1] != 3:
            raise ValueError(f"Expected 3D points with shape (N, 3), got shape {points.shape}")

        # Identity rotation and zero translation (points already in the camera frame)
        rvec = np.zeros((3, 1), dtype=np.float32)
        tvec = np.zeros((3, 1), dtype=np.float32)

        # Project all points
        points_projected, _ = cv2.projectPoints(points, rvec, tvec, camera_matrix, None)

        # Flatten to get 2D points with shape (N, 2)
        return points_projected.reshape(-1, 2)
    

    
    def PnP_from_holes(self, holes: list[Hole], T: SE3) -> tuple[bool, SE3 | None]:

        """
        Perform a PnP pose estimation using 2 lists of holes.
        At the same index, in the 2 lists, corresponds the same one hole: once described in camera RF, and once in hole RF
        """

        # 3D points of the holes in flange reference frame (object points)
        centers_holes_flangeRF = self.get_holes_centers(holes, "flange", only_with_camera_repr = True)

        # 2D points of detected holes projected onto the image (image points)
        centers_holes_cameraRF_projected = np.array([self.project_3D_points_on_screen(hole["camera"].center, self.camera_params.CAMERA_MATRIX).ravel()
                                           for hole in holes if hole["camera"].center is not None])

        # Run PnP to estimate pose
        success, self.T = self.custom_PnP(centers_holes_flangeRF, centers_holes_cameraRF_projected, self.camera_params.CAMERA_MATRIX, T)

        if not success:
            if self.debug_params.DETAILS: print("-[FAIL] Could not solve final PnP")
            if self.debug_params.DRAW_RESULTS: self.image.images_with_drawings.append(self.image.frame)
            return success, None
        
        return success, self.T



    def reprojection_error_check(self, T: SE3, holes: list[Hole]) -> bool:
        """
        Projects 3D points on the screen, using the calculated R,t and accept the solution
        only if every projected point is closer to the corresponding image point than "TRESHOLD_REFINED_REPROJECTION_ERROR"
        """

        # 3D points of the holes in flange reference frame (object points)
        centers_holes_flangeRF = self.get_holes_centers(holes, "flange", only_with_camera_repr = True)
        # 2D points of detected holes projected onto the image (image points)
        centers_holes_cameraRF_projected = np.array([self.project_3D_points_on_screen(hole["camera"].center, self.camera_params.CAMERA_MATRIX).ravel()
                                           for hole in holes if hole["camera"].center is not None])
        
        # Check final reprojection error
        projection_errors = self.get_projection_distances_list(centers_holes_cameraRF_projected, centers_holes_flangeRF, T, self.camera_params.CAMERA_MATRIX )

        # Print detailed reprojection errors
        if self.debug_params.MORE_DETAILS:
            for idx, err in enumerate(projection_errors):
                print(f"-[MORE DETAILS] Refined pose reprojection error = {float(err):.3f} [pix]")

        # Pose refinement passes only if all points are below threshold
        success = np.all(projection_errors[:, 0] <= self.pose_estimation_params.TRESHOLD_REFINED_REPROJECTION_ERROR)

        if not success:
            if self.debug_params.DETAILS:
                print("-[FAIL] Could not pass final reprojection error test")
            if self.debug_params.DRAW_RESULTS:
                self.image.images_with_drawings.append(self.image.frame)
                self.image.images_with_drawings.append(self.image.frame)
                
        return success
    


    def make_pose_estimation_draw(self) -> None:
        """
        Draw the final pose estimation results on the image for visualization.
        """
        if not self.success:
            self.image.images_with_drawings.append(self.image.frame)
            self.image.images_with_drawings.append(self.image.frame)        
                
        self.image.draw.pose_axes(self.T, self.image.images_with_drawings[1])
        self.image.draw.PnP(self.T, self.holes, self.image.images_with_drawings[2])



    def get_pose(self) -> tuple[bool, SE3 | None]:
        """
        Refine the pose using PnP and reprojection error validation.
        """

        self.success, self.T = self.PnP_from_holes(self.holes, self.T)
        if not self.success: return self.success, None

        self.success = self.reprojection_error_check(self.T, self.holes)
        if not self.success: return self.success, None
                
        return self.success, self.T
    


