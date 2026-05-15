import numpy as np  

from ur_alignment.flange_pose_estimator.pose_computation.pose_computation_core import PoseEstimatorCore
from ur_alignment.flange_pose_estimator.utility_classes import Hole


class PoseEvaluator(PoseEstimatorCore):

    def __init__(self, pose_estimator: PoseEstimatorCore) -> None:

        super().__init__(image = pose_estimator.image, 
                         T = pose_estimator.T,
                         params = pose_estimator.params,
                         holes = pose_estimator.holes
        )
        self.success = pose_estimator.success


    def convert_matching_indices_to_data(self, 
                                         matches: list[list[int | float]],
                                         points1: np.ndarray, points2: np.ndarray
                                        ) -> tuple[list[list[np.ndarray]], np.ndarray]:
        """
		Convert index-based matches to actual point-pair data with distances.
		"""   
        matching_points = []
        matching_distances = np.array([])
        for match in matches:
            i, j, distance = match
            point1 = points1[i]
            point2 = points2[j]

            matching_distances = np.append(matching_distances, distance)
            matching_points.append([point1, point2])

            if self.params.debug_params.MORE_DETAILS: 
                print(f"-[MORE DETAILS] Homography error = {1000* distance:.4} [mm]")

        return matching_points, matching_distances
    


    def get_translation_correction(self, hole: Hole) -> np.ndarray:
        """
        Correct the translation vector if the hole is at a different crown height.
        """
        crown_height = hole["flange"].center[2] # z-coordinate
        translation_local = np.array([0, 0, crown_height])

        translation_global = self.R @ translation_local.reshape(3, 1)
        tvec_corrected = self.tvec + translation_global
        return tvec_corrected



    def get_homography(self, tvec_corrected: np.ndarray) -> np.ndarray:
        """
        Compute the homography matrix for projecting from flange plane to image.
        """
        # Pose of the homography plane relative to the camera plane
        T_cf = np.vstack((np.hstack((self.R, tvec_corrected)),
                          np.array( [0, 0, 0, 1])))

        C = np.hstack((np.eye(3), np.zeros((3,1))))
        # Full projection matrix
        P = self.K @ C @ T_cf
        # Homography matrix
        H = np.delete(P, 2, axis=1)

        return H



    def get_center_flange_projected(self, H: np.ndarray, center_ellipse_homog: np.ndarray) -> np.ndarray:
        """
        Project an image center back to the flange plane using inverse homography.
        """
        # Invert the homography
        H_inv = np.linalg.inv(H)

        # Map points from image back to flange plane
        center_flange_homog = (H_inv @ center_ellipse_homog.T).T
        center_flange_homog /= center_flange_homog[2]
        center_flange = center_flange_homog[:2]
        return center_flange      



    def evaluate_pose(self) -> bool:
        """
        Evaluate the quality of the estimated pose via homography-based matching.
        """
        if not self.success:
            if self.debug_params.DETAILS: print("-[DETAIL] Tried to evaluate pose result, but no correct pose was found yet")
            return None
        
        # camera matrix
        self.K = self.camera_params.CAMERA_MATRIX
        
        # Pose extraction
        self.R = self.T.R
        self.tvec = self.T.t.reshape(3,1)
    
        centers_flange = []
        for hole in self.holes:
            
            # Get the center of the ellipse on the plane ----> only if it passed every check
            if hole["camera"].center is None: continue
            center_ellipse_homog = self.make_homogeneous(np.array([hole.projection_edge.cx, hole.projection_edge.cy]))

            # Get the homography that maps the flange plane on the image plane, corrected if the model point is not on the same height of the RF
            tvec_corrected = self.get_translation_correction(hole)
            H = self.get_homography(tvec_corrected)

            # Get the 2D coordinates of the image plane projected on the flange homography plane
            center_flange = self.get_center_flange_projected(H, center_ellipse_homog)
            center_flange = centers_flange.append(center_flange)

        centers_flange = np.array(centers_flange)
        for center_flange in centers_flange: 
            center_flange = np.append(center_flange, 1)

        centers_holes = self.get_holes_centers(self.holes, "flange")[:, :2]

        matches_indices = self.get_sorted_matches_by_distance(centers_flange, centers_holes)
        self.matching_points, self.matching_distances = self.convert_matching_indices_to_data(matches_indices, centers_flange, centers_holes)
        self.success = self.evaluate_matching_distances()

        return self.success
    

    def evaluate_matching_distances(self) -> bool:
        """
        Validate that homography errors are below the specified threshold.
        """
        max_distance = np.max(self.matching_distances)
        THRESHOLD = self.params.pose_estimation_params.TRESHOLD_HOMOGRAPHY_DISTANCE
        success = max_distance < THRESHOLD

        if self.params.debug_params.DETAILS:
            if success: print(f"-[SUCCESS] Pose estimated with maximum homography projection error = {1000*float(max_distance):.4} [mm]")

            else: print(f"-[FAIL] Pose estimated with maximum homography projection error = {1000*float(max_distance):.4} [mm] > {1000*THRESHOLD} [mm]")

        return success



    def get_matching_points(self) -> list[list[np.ndarray]]:
        """
        Return the list of matched point pairs used for pose evaluation.
        """
        if self.matching_points is None:
            raise ValueError("No analysis has been conducted yet. Cannot get any matching point")
        return self.matching_points
    


    def get_matching_distances(self) -> np.ndarray:
        """
        Return the list of distances used in the homography error analysis.
        """
        if self.matching_distances is None:
            raise ValueError("No analysis has been conducted yet. Cannot get any matching distance")
        return self.matching_distances


