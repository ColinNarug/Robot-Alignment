import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import copy
from spatialmath import SE3


from ur_alignment.flange_pose_estimator.utility_classes import Params, Image, Hole, Ellipse



class PoseEstimatorCore():

    def __init__(self, 
                params: Params,
                image: Image = None,
                T: SE3 = None,
                T_past: SE3 = None,
                holes: list[Hole] = None
                ):

        # parameters
        self.params = params
        self.camera_params = params.camera_params
        self.flange_model_params = params.flange_model_params
        self.pose_estimation_params = params.pose_estimation_params
        self.debug_params = params.debug_params
        
        # Image plane info
        self.image = image

        # Current pose (R,t) of the flange relative to the camera RF
        self.T = T

        # Past pose (R,t) of the flange relative to the camera RF
        self.T_past = T_past       # 3x3 rotation matrix (np.ndarray)

        # Holes reconstructed in flange RF and camera RF
        self.holes = copy.deepcopy(holes) # List of Hole objects 



    @staticmethod
    def get_holes_centers(holes: list[Hole], reference_frame: str, only_with_camera_repr: bool = False, only_with_projection: bool = False) -> np.ndarray:
        """
        From a list of holes, get the corresponding list of centers
        """
        centers = []
        for hole in holes :

            if only_with_camera_repr and hole["camera"].center is None:
                continue

            if only_with_projection and hole.projection_edge is None:
                continue

            centers.append(hole[reference_frame].center)

        return np.array(centers)
    


    @staticmethod
    def get_holes_projections_centers(holes: list[Hole], only_with_camera_repr: bool = False) -> np.ndarray:
        """
        From a list of ellipses, get the corresponding list of centers
        """
        if only_with_camera_repr:
            centers = np.array([[hole.projection_edge.cx, hole.projection_edge.cy] for hole in holes if hole["camera"].center is not None and hole.projection_edge is not None]) 
            return centers

        centers = np.array([[hole.projection_edge.cx, hole.projection_edge.cy] for hole in holes if hole.projection_edge is not None]) 
        return centers



    @staticmethod
    def get_ellipses_centers(ellipses: list[Ellipse]) -> np.ndarray:
        """
        From a list of ellipses, get the corresponding list of centers
        """
        centers = np.array([[ellipse.cx, ellipse.cy] for ellipse in ellipses])
        return centers
    


    def project_holes_on_image(self, holes: list[Hole], T: SE3, CAMERA_MATRIX: np.ndarray) -> np.ndarray:
        """
        Project the holes in flange RF on the image plane
        """

        tvec = T.t
        R = T.R
        rvec, _ = cv2.Rodrigues(R)

        centers_holes = self.get_holes_centers(holes, "flange")    # objpoints
        # project the objectpoints
        centers_holes_projected, _ = cv2.projectPoints(centers_holes, rvec, tvec, CAMERA_MATRIX, None)
        centers_holes_projected = np.int32(centers_holes_projected).reshape(-1, 2)

        return centers_holes_projected



    @staticmethod
    def _restore_holes_without_projection(holes: list[Hole], holes_original: list[Hole]) -> list[Hole]:
        """Order-stable merge: keep current holes; restore only originals without projection."""
        by_id = {int(h.ID_model): h for h in holes}

        restored: list[Hole] = []
        for h0 in holes_original:
            cur = by_id.get(int(h0.ID_model))
            if cur is not None:
                restored.append(cur)  # keep current (possibly updated) hole
            else:
                # restore only if original had no projection (avoid re-adding rejected projected holes)
                if getattr(h0, "projection_edge", None) is None:
                    restored.append(h0)
                # else: skip (it was a projected hole that was filtered out)

        return restored


    @staticmethod
    def get_projection_distances_list(imgpoints: np.ndarray, objpoints: np.ndarray, T: SE3, CAMERA_MATRIX: np.ndarray) -> np.ndarray:
        """
        1. Project each objectpoint on the image plane, using R, tvec, CAMERA_MATRIX
        2. Calculate the distances between each imagepoint and each corresponding projected objectpoint
        3. If the distance is greater than the threshold, append [distance, False], if is lesser, append [distance, True]
        """
        imgpoints = np.asarray(imgpoints, dtype=np.float32).reshape(-1, 2)
        objpoints = np.asarray(objpoints, dtype=np.float32).reshape(-1, 3)

        if imgpoints.size == 0 or objpoints.size == 0:
            return np.empty((0, 1), dtype=np.float32)
        if len(imgpoints) != len(objpoints):
            raise ValueError(f"imgpoints ({len(imgpoints)}) and objpoints ({len(objpoints)}) mismatch")

        rvec, _ = cv2.Rodrigues(T.R)
        tvec = T.t.astype(np.float32).reshape(3, 1)

        projected, _ = cv2.projectPoints(objpoints, rvec, tvec, CAMERA_MATRIX, None)
        projected = projected.reshape(-1, 2).astype(np.float32)  # niente int32!

        dists = np.linalg.norm(projected - imgpoints, axis=1).astype(np.float32)
        return dists.reshape(-1, 1)



    @staticmethod
    def _ZX_axis_flip(R_old: np.ndarray) -> np.ndarray:
        """
        Flip Z, X axis of the rotation matrix, and recalculate Y axis with the cross product
        """
        # Flip the Z axis 
        z_old = R_old[:, 2]
        z_new = -z_old
        x_old = R_old[:, 0]
        x_new = -x_old
        y_new = np.cross(z_new, x_new)
        R_new = np.column_stack((x_new, y_new, z_new))
        return R_new



    def _holes_axis_flip_adjustment(self, holes: list[Hole], R_new: np.ndarray, R_old: np.ndarray, z_pointing_inward: bool) -> list[Hole]:
        """
        Transform the holes to the new flange pose RF
        """
        R_cam_old = R_old                   # Old flange pose relative to the camera RF 
        R_cam_new = R_new                   # New flange pose relative to the camera RF
        R_new_old = R_cam_new.T @ R_cam_old # Old flange pose relative to the new flange pose RF

        adjust_height = False
        if self.flange_model_params.NUM_UNIQUE_HEIGHTS == 1 and z_pointing_inward:
            adjust_height = True
            height = self.flange_model_params.CROWNS[0].HEIGHT 
            translation_flangeRF = np.array([0, 0, 2*height])
            translation_cameraRF = - R_new_old @ translation_flangeRF

        # Transform the holes to the new flange pose RF
        new_holes = []
        for hole in holes:
            new_hole = Hole()
            # ------- Base ------- #
            new_hole.ID = hole.ID
            new_hole.radius = hole.radius
            new_hole.ID_model = hole.ID_model
            new_hole.projection_edge = hole.projection_edge
            # ------- Flange RF ------- #
            new_hole["flange"].center = R_new_old @ hole["flange"].center
            if adjust_height: new_hole["flange"].center = new_hole["flange"].center + translation_cameraRF
            new_hole["flange"].normal = R_new_old @ hole["flange"].normal
            # ------- Camera RF ------- #
            new_hole["camera"].center = hole["camera"].center
            new_hole["camera"].normal = hole["camera"].normal

            # Append
            new_holes.append(new_hole)

        return new_holes
    


    @staticmethod
    def _find_closest_orientation(R: np.ndarray, R_past: np.ndarray, angle_diff_threshold: float) -> np.ndarray:
        """
        Keeps the orientation consistent across frames by selecting the closest rotation
        to the previous one among discrete rotations around the z-axisin steps of angle_diff_threshold

        angle_diff_threshold (float)  --> [deg] is the smallest rotation around the flange z-axis that maps
                                          any simmetry plane of the flange in another simmmetry plane
                                          
        Essentially, it's the smallest angle that has such property: given any z-axis rotations of "theta" 
        degrees of the flange, in two adjacent frames, it's not possible to tell if the flange is rotated by 
        "theta", or by "theta + angle_diff_threshold", or by "theta + 2*angle_diff_threshold"...
        """        

        if R_past is None: return R
        R = R.copy()
        R_past = R_past.copy()

        angle_diff_threshold_deg = angle_diff_threshold * 180 / np.pi # convert to degrees
        angles_deg = np.arange(0, 360, angle_diff_threshold_deg) # angles in degrees: step = 22.5°
        
        candidates = []
        for angle in angles_deg:
            R_z = Rotation.from_euler('z', angle, degrees=True).as_matrix()
            candidate = R @ R_z
            candidates.append(candidate)

        best_angle = np.inf
        R_consistent = None
        R_past_inv = R_past.T 

        for candidate in candidates:
            R_diff = R_past_inv @ candidate
            trace_val = np.clip((np.trace(R_diff) - 1) / 2.0, -1.0, 1.0)
            angle_diff = np.arccos(trace_val)
            if angle_diff < best_angle:
                best_angle = angle_diff
                R_consistent = candidate

        return R_consistent



    @staticmethod
    def _posizion_axis_flip(R: np.ndarray, tvec_old: np.ndarray, height: float) -> np.ndarray:

        translation_flangeRF = np.array([0, 0, 2*height]).reshape(3,1)
        
        translation_cameraRF = R @ translation_flangeRF
        tvec_new = tvec_old - translation_cameraRF
        return tvec_new


  
    def adjust_pose_axis(self, T: SE3, T_past: SE3 = None, holes: list[Hole] = None) -> tuple[SE3, list[Hole] | None]:
        """
        Keeps the Z axis of the flange pointing outward the flange
        """
        T_not_adjust = copy.deepcopy(T)
        R_not_adjust = T_not_adjust.R
        tvec_not_adjust = T_not_adjust.t.reshape(3,1)

        # Check if the Z axis is pointing inward
        z_not_adjust = R_not_adjust[:, 2]
        z_pointing_inward = (z_not_adjust[2] > 0) # Z axis pointing inward ?
        if z_pointing_inward:  
            R = self._ZX_axis_flip(R_not_adjust) 

            if self.flange_model_params.NUM_UNIQUE_HEIGHTS == 1:
                height = self.flange_model_params.CROWNS[0].HEIGHT 
                tvec = self._posizion_axis_flip(R, tvec_not_adjust, height)
            else: tvec = tvec_not_adjust
        else: 
            R = R_not_adjust
            tvec = tvec_not_adjust

        # Find the closest orientation to the past pose
        if T_past is not None: 
            R_past = T_past.R
            angle_step_model = self.flange_model_params.CROWNS[0].ANGLE_STEP
            R = self._find_closest_orientation(R, R_past, angle_step_model)

        # Adjust the holes to the new flange pose RF
        if holes is not None: 
            holes = self._holes_axis_flip_adjustment(holes, R, R_not_adjust, z_pointing_inward)

        T = SE3.Rt(R, tvec)
        return T, holes



    @staticmethod
    def custom_PnP(objpoints: np.ndarray, imgpoints: np.ndarray, camera_matrix: np.ndarray, T_init: SE3 = None) -> tuple[bool, SE3 | None]:

        # Choose parameters
        chosen_method = cv2.SOLVEPNP_ITERATIVE
        refinement_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-4)
        use_initial_solution = False

        # Prepare initial guesses
        if T_init is not None:
            R_init = T_init.R
            rvec_init, _ = cv2.Rodrigues(R_init)
            tvec_init = T_init.t
        else:
            rvec_init = None
            tvec_init = None

        if (rvec_init is None or tvec_init is None):
            use_initial_solution = False

        objpoints = np.asarray(objpoints, dtype=np.float32).reshape(-1, 3)
        imgpoints = np.asarray(imgpoints, dtype=np.float32).reshape(-1, 2)

        # Ensure initial guesses are correctly shaped
        if rvec_init is not None: rvec_init = np.asarray(rvec_init, dtype=np.float64).reshape(3, 1)
        if tvec_init is not None: tvec_init = np.asarray(tvec_init, dtype=np.float64).reshape(3, 1)

        # SolvePnP
        success, rvec, tvec = cv2.solvePnP(
            objpoints, imgpoints, 
            camera_matrix, None, 
            rvec_init, tvec_init, useExtrinsicGuess = use_initial_solution,
            flags = chosen_method
        )

        if not success: return False, None

        # Refine
        rvec, tvec = cv2.solvePnPRefineLM(
            objpoints, imgpoints, 
            camera_matrix, None, 
            rvec, tvec, 
            criteria = refinement_criteria
        )

        R, _ = cv2.Rodrigues(rvec)
        T = SE3.Rt(R, tvec)

        return success, T
    


    @staticmethod
    def get_distance_matrix(imgpts1: np.ndarray, imgpts2: np.ndarray) -> np.ndarray:
        return np.linalg.norm( imgpts1[:, None, :] - imgpts2[None, :, :], axis=2)
    


    @staticmethod
    def check_matches_within_threshold(matches: list[list[int | float]], threshold: float) -> bool:
        for _, _, distance in matches:
            if distance > threshold:
                return False
        return True


    @staticmethod
    def get_sorted_matches_by_distance(imgpts1: np.ndarray, imgpts2: np.ndarray, debug: bool = False) -> list[list[int | float]]:
        distances_matrix = np.linalg.norm(imgpts1[:, None] - imgpts2[None, :], axis=2)
        row_ids = np.arange(imgpts1.shape[0])
        col_ids = np.arange(imgpts2.shape[0])
        matches = []

        while distances_matrix.size > 0:
            min_idx = np.unravel_index(np.argmin(distances_matrix), distances_matrix.shape)
            min_distance = distances_matrix[min_idx]

            if debug: print(f"-[MORE DETAILS] Image point pair distance: {min_distance:.3f}")

            i_original = row_ids[min_idx[0]].astype(int)
            j_original = col_ids[min_idx[1]].astype(int)
            matches.append([i_original, j_original, min_distance])

            distances_matrix = np.delete(distances_matrix, min_idx[0], axis=0)
            distances_matrix = np.delete(distances_matrix, min_idx[1], axis=1)
            row_ids = np.delete(row_ids, min_idx[0])
            col_ids = np.delete(col_ids, min_idx[1])

        return matches
    

    @staticmethod
    def make_homogeneous(points: np.ndarray | list) -> np.ndarray:
        """
        Converts 2D or 3D points into homogeneous coordinates.
        Accepts a single point or an array of points.
        """
        points = np.asarray(points)
        
        if points.ndim == 1:
            if points.shape[0] not in [2, 3]:
                raise ValueError("A single point must have 2 or 3 coordinates.")
            ones = np.array([1.0])
            homogeneous_points = np.hstack([points, ones])
        elif points.ndim == 2:
            if points.shape[1] not in [2, 3]:
                raise ValueError("Points must have 2 or 3 coordinates each.")
            ones = np.ones((points.shape[0], 1))
            homogeneous_points = np.hstack([points, ones])
        else:
            raise ValueError("Input must be a 1D or 2D array.")
        
        return homogeneous_points



    @staticmethod
    def make_unhomogeneous(points: np.ndarray | list) -> np.ndarray:
        """
        Converts homogeneous points back to non-homogeneous coordinates.
        Accepts a single point or an array of points.
        """
        points = np.asarray(points)
        
        if points.ndim == 1:
            if points.shape[0] not in [3, 4]:
                raise ValueError("A single homogeneous point must have 3 or 4 coordinates.")
            return points[:-1] / points[-1]
        elif points.ndim == 2:
            if points.shape[1] not in [3, 4]:
                raise ValueError("Homogeneous points must have 3 or 4 coordinates each.")
            return points[:, :-1] / points[:, -1, np.newaxis]
        else:
            raise ValueError("Input must be a 1D or 2D array.")