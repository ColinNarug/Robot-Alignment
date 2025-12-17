import numpy as np
import cv2
from spatialmath import SE3

from .matcher_smart import MatcherSmart
from .matcher_brute_force import MatcherBruteForce
from .pose_computation_core import PoseEstimatorCore
from ur_alignment.flange_pose_estimator.utility_classes import Hole, Ellipse




class PoseInitializer(PoseEstimatorCore):

    def __init__(self, pose_estimator: PoseEstimatorCore) -> None:

        super().__init__(image = pose_estimator.image, 
                         T = pose_estimator.T,
                         T_past = pose_estimator.T_past, 
                         params = pose_estimator.params
                         )
        
        # Original set of holes, extraced from the model
        self.holes_original = pose_estimator.holes_original.copy()
        # Initialization succes
        self.success = False
                


    def custom_PnP_holes_based(self, holes: list[Hole], T: SE3 = None) -> tuple[bool, SE3 | None]:
        """
        Estimate pose using hole projections and optionally a prior guess.
        """
        ellipses_centers = self.get_holes_projections_centers(holes)                          # imagepoints
        centers_holes = self.get_holes_centers(holes, "flange", only_with_projection = True)  # objpoints

        success, T = self.custom_PnP(centers_holes, ellipses_centers, self.camera_params.CAMERA_MATRIX, T)

        return success, T
    


    def get_dict_by_ID(self, objects: list) -> dict[int, object]:
        """
        Build a dictionary mapping IDs to objects, with placeholders for unassigned.
        """
        last_unassigned_key = -2

        objects_dict = {} 
        for object in objects:
            if object.ID != -1 and object.ID is not None:
                objects_dict[int(object.ID)] = object
            else:
                objects_dict[last_unassigned_key] = object
                last_unassigned_key -= 1
        return objects_dict



    def _get_tracking_matches(self, past_tracking_data: dict[int, np.ndarray], ellipses_dict: dict[int, Ellipse]) -> bool:
        """
        Verify that tracked ellipses match expected past positions.
        """
        TRESHOLD = self.params.pose_estimation_params.TRESHOLD_TRACKING_REPROJECTION_ERROR

        # keep only the positive IDs (the ones actually tracked)
        # current -> {IDs: centers_coordinates}
        curr = {ID: np.array([e.cx, e.cy], dtype=np.float32)
                for ID, e in ellipses_dict.items() if ID is not None and ID > 0}
        # past    -> {IDs: centers_coordinates}
        past = {ID: np.array(past_tracking_data[ID], dtype=np.float32)
                for ID in past_tracking_data.keys() if ID is not None and ID > 0}

        if len(past) == 0 or len(curr) == 0: return False
        
        # Common IDs in the current and past tracking data
        common = set(curr.keys()) & set(past.keys())
        if len(common) == 0:
            if self.debug_params.MORE_DETAILS:
                print("-[MORE DETAILS] Tracking failed: no common IDs between frames")
            return False

        # corresponding IDs centers distances
        curr_arr = np.stack([curr[ID] for ID in common])
        past_arr = np.stack([past[ID] for ID in common])
        distances = np.linalg.norm(curr_arr - past_arr, axis=1)

        # if some distance is grather than TRESHOLD -> tracking off
        over = distances > TRESHOLD
        if np.any(over):
            if self.debug_params.MORE_DETAILS:
                common_list = list(common)
                bad_id = common_list[int(np.argmax(distances))]
                print(f"-[MORE DETAILS] Tracking failed: ID {bad_id} moved {distances.max():.1f}px > {TRESHOLD}")
            return False

        if self.debug_params.MORE_DETAILS:
            print(f"-[MORE DETAILS] Maximum tracking distance = {float(distances.max()):.1f}[pix]")
        return True



    def _assign_projection_by_projecting(self, holes_unasigned_dict: dict[int, Hole], ellipse: Ellipse, rvec: np.ndarray, tvec: np.ndarray) -> tuple[int | None, bool]:
        """
        Assign a projected 3D hole to a 2D ellipse based on distance.
        """
        success = False
        best_index = None

        ellipse_center = np.array([ellipse.cx, ellipse.cy])
        best_distance = self.params.pose_estimation_params.TRESHOLD_TRACKING_REPROJECTION_ERROR
        
        for ID, hole in holes_unasigned_dict.items():
            center = hole["flange"].center
            hole_center_projected, _ = cv2.projectPoints(center, rvec, tvec, self.params.camera_params.CAMERA_MATRIX, None)
            hole_center_projected = hole_center_projected.reshape(-1, 2)
            distance = np.linalg.norm(ellipse_center - hole_center_projected)
            
            if best_distance > distance:
                best_distance = distance
                best_index = ID
                success = True

        if self.params.debug_params.MORE_DETAILS: 
            if best_index is None:
                print(f"-[MORE DETAILS] racking failed")
            else:
                print(f"-[MORE DETAILS] Initialization with manual reprojection: best distance = {best_distance:.3} [pix] -> for ID = {best_index}")

        return best_index, success
            


    def _get_holes_unasigned_dict(self, holes_dict: dict[int, Hole]) -> dict[int, Hole]:
        """
        Extract holes without valid assigned IDs from dictionary.
        """       
        holes_unasigned_dict = {}
        for ID, hole in holes_dict.items():
            if ID < -1:
                holes_unasigned_dict[ID] = hole
        return holes_unasigned_dict




    def _clean_old_projections(self, holes_dict: dict[int, Hole], IDs_with_projection: list[int]) -> dict[int, Hole]:
        """
        Clear projections from holes not assigned in the current frame.
        """
        for ID_dict, hole in holes_dict.items():
            if ID_dict not in IDs_with_projection:
                hole.projection_edge = None
        return holes_dict




    def _assign_projections_to_holes(self, holes_dict: dict[int, Hole], ellipses_dict: dict[int, Ellipse], T_past: SE3) -> tuple[bool, list[Hole]]:
        """
        Assign the best matching ellipses to holes based on proximity.
        """
        
        # 0) pulizia totale: niente ghost
        for h in holes_dict.values():
            h.projection_edge = None

        holes_unasigned_dict = self._get_holes_unasigned_dict(holes_dict)

        rvec, _ = cv2.Rodrigues(T_past.R)
        tvec = T_past.t

        success = True
        assigned_count = 0

        for ID, ellipse in ellipses_dict.items():
            if ID in holes_dict:  # stesso ID foro -> assegnazione diretta
                holes_dict[ID].projection_edge = ellipse
                assigned_count += 1
                continue

            # prova a proiettare per trovare il foro "giusto" tra i non assegnati
            ID_hole, ok = self._assign_projection_by_projecting(holes_unasigned_dict, ellipse, rvec, tvec)
            if not ok:
                success = False
                break
            holes_dict[ID_hole].projection_edge = ellipse
            assigned_count += 1

        holes = list(holes_dict.values())

        return success, holes



    def _active_tracking_check(self, past_tracking_data: dict[int, np.ndarray], T_past: SE3, holes: list[Hole], ellipses: list[Ellipse]) -> tuple[bool, list[Hole]]:
        """
        Check if active tracking is valid and assign projections accordingly.
        """

        if past_tracking_data is None or T_past is None: return False, None

        # {ID1 : ellipse_1 ; ID2 : ellipse_2 ; ...}
        ellipses_dict = self.get_dict_by_ID(ellipses) 
        # {ID1 : hole_1 ; ID2 : hole_2 ; ...}
        holes_dict = self.get_dict_by_ID(holes) 

        # Tracking current status
        success = self._get_tracking_matches(past_tracking_data, ellipses_dict)
        if not success: return False, holes

        success, holes  = self._assign_projections_to_holes(holes_dict, ellipses_dict, T_past)
        return success, holes
    


    @staticmethod
    def _assign_projections_by_indices(sorting_indices: list[list[int]], holes: list[Hole], ellipses: list[Ellipse]) -> list[Hole]:
        """
        Assign ellipses to holes using matching index pairs.
        """

        sorting_indices_dict = {}
        for indices in sorting_indices:
            sorting_indices_dict[indices[0]] = indices[1]

        holes_assigned = []
        for index in range(len(holes)):

            next_hole = Hole()
            next_hole.ID_model = holes[index].ID_model
            next_hole.radius = holes[index].radius
            next_hole["flange"].center = holes[index]["flange"].center
            next_hole["flange"].normal = holes[index]["flange"].normal

            if index not in sorting_indices_dict.keys():
                holes_assigned.append(next_hole)
                continue

            ellipse_index = sorting_indices_dict[index]
            next_hole.ID = ellipses[ellipse_index].ID
            next_hole.projection_edge = ellipses[ellipse_index]
            holes_assigned.append(next_hole)

        return holes_assigned




    def _get_tracking_status(self, past_refinement_data: dict[int, np.ndarray]) -> str:
        """
        Check if the tracking is preserved by projecting the model point, using (T_past)
        
        --> If so, use the past pose (T_past) as the coarse pose estimate ==> STATUS = "active_tracking"
        
        --> If not, start the initialization of the pose estimate (T_init).
            To initialize assume hole center of the projected ellipse (ellipse_center), is the same
            as the projection of the 3d hole center (center_holes_model -> projected)

            ----> If the flange has only one holes crown: use the "SmartHolesProjectionsMatcher", then perform a PnP
                    ==> STATUS = "smart_recovery"
            ----> If the flange has more holes crowns: use the "BruteForcePoseInitializer" (iterative PnP)
                    ==> STATUS = "brute_force_recovery"
        """
        self.success, self.holes = self._active_tracking_check(past_refinement_data, self.T_past, self.holes, self.image.ellipses)
        if self.success: return "active_tracking"
            
        if self.flange_model_params.TOTAL_NUM_CROWNS >= 2:
            return "brute_force_recovery"

        return "smart_recovery"



    def match_holes_model_and_projections(self, holes: list[Hole], past_refinement_data: dict[int, np.ndarray]) -> None:
        """
        Match the holes in model RF with the projections of the holes on in the image.
        """

        self.holes = holes
        tracking_status = self._get_tracking_status(past_refinement_data)

        self.tracking_status = tracking_status
        if self.debug_params.DETAILS: print(f"-[DETAILS] Tracking status: {tracking_status}")

        if tracking_status == "active_tracking":
            # If the tracking is active cannot fail
            self.T = self.T_past


        if tracking_status == "smart_recovery":
            # Initialize the holes set
            self.holes = self.holes_original.copy()
            # Use the MatcherSmart to match the holes and the projections
            smart_holes_projections_matcher = MatcherSmart(self, self.holes)
            self.holes = smart_holes_projections_matcher.match_holes_and_projections() 
            self.success, self.T = self.custom_PnP_holes_based(self.holes)
            

        if tracking_status == "brute_force_recovery":
            # Initialize the holes set
            self.holes = self.holes_original.copy()
            # Use the MatcherBruteForce to match the holes and the projections
            self.bruteforce_holes_projections_matcher = MatcherBruteForce(self, self.holes)
            self.success, self.holes = self.bruteforce_holes_projections_matcher.match_holes_and_projections() 
            self.T = self.bruteforce_holes_projections_matcher.get_pose()
 

        # In the end, adjust the pose
        if self.success:          
            self.T, self.holes = self.adjust_pose_axis(self.T, self.T_past, self.holes)



    def get_initial_pose(self) -> tuple[bool, SE3 | None]:
        if not self.success and self.debug_params.DETAILS:
            print("-[FAIL] Initialization failed.")                

        return self.success, self.T
        

    
    
    def get_holes(self) -> list[Hole]:
        return self.holes
    
    