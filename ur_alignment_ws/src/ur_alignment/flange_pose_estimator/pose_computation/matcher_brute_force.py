import numpy as np
import cv2
import itertools
from spatialmath import SE3
from typing import Optional

from ur_alignment.flange_pose_estimator.pose_computation.pose_computation_core import PoseEstimatorCore




class MatcherBruteForce(PoseEstimatorCore):

    def __init__(self, pose_estimator: PoseEstimatorCore, holes: list) -> None:

        super().__init__(image = pose_estimator.image, 
                         T_past = pose_estimator.T_past, 
                         params = pose_estimator.params,
                         holes = holes)
        

    def get_pose(self) -> Optional[SE3]:
        if not self.success: return None
        return self.T
    


    @staticmethod
    def sort_set_by_indices(set_unsorted: list, sort_indices: list[int]) -> list:
        """
		Sort elements of a list using the provided indices.
		"""
        set_sorted = []

        for sort_index in sort_indices:
            set_sorted.append(set_unsorted[int(sort_index)])

        return set_sorted
    

    
    def check_PnP(self, R: np.ndarray, tvec: np.ndarray,
                model_centers_remaining: np.ndarray, centers_ellipses_remaining: np.ndarray,
                threshold: float) -> tuple[bool, np.ndarray]:
        """
		Project remaining model points and match them with image points using a threshold.
		"""
        projected_model_centers_remaining, _ = cv2.projectPoints(model_centers_remaining, R, tvec, self.camera_params.CAMERA_MATRIX, None)
        projected_model_centers_remaining = projected_model_centers_remaining.reshape(-1, 2)
        
        matches = self.get_sorted_matches_by_distance(projected_model_centers_remaining, centers_ellipses_remaining)
        # check if distance_ij < threshold for each i,j
        success = self.check_matches_within_threshold(matches, threshold)
        matches = np.array(matches) # cast to int to be used as list indices

        return success, matches
    

    @staticmethod
    def split_set(set: np.ndarray, selected_indices: list[int]) -> tuple[np.ndarray, np.ndarray]:
        """
		Split a set into selected and remaining elements using index selection.
		"""
        if len(set) < len(selected_indices):
            raise ValueError("The desired size of the smaller set is larger than the size of the whole set")

        selected_set = set[selected_indices]
        mask = np.ones(len(set), dtype=bool)
        mask[selected_indices] = False
        remaining_set = set[mask]
        return selected_set, remaining_set



    @staticmethod
    def calculate_score(matches: list) -> float:
        """
		Compute the average distance score from a list of matches.
		"""
        score = np.mean([match[2] for match in matches])
        return score
    


    def sort_holes_and_projections(
        self, matches: np.ndarray, selected_indices: np.ndarray,
        remaining_indices: list[int], selected_indices_ellipses: np.ndarray
    ) -> None:
        """
        Reorder holes and ellipses to match by index and assign projections to holes.
        """

        mapping = {ell: hole for ell, hole in zip(selected_indices_ellipses, selected_indices)}

        #    matches:  (idx_hole_in_remaining, idx_ellipse_in_remaining, dist)
        base_ellipse_idx = max(selected_indices_ellipses) + 1  # cioè 4
        for idx_hole_r, idx_ell_r, _ in matches:
            ellipse_idx = base_ellipse_idx + idx_ell_r
            hole_idx    = remaining_indices[int(idx_hole_r)]
            mapping[ellipse_idx] = hole_idx

        ordered_pairs   = sorted(mapping.items())      
        ellipses_idx    = [p[0] for p in ordered_pairs]
        holes_idx       = [p[1] for p in ordered_pairs]

        self.holes            = self.sort_set_by_indices(self.holes, holes_idx)
        self.image.ellipses   = self.sort_set_by_indices(self.image.ellipses, ellipses_idx)

        for hole, proj in zip(self.holes, self.image.ellipses):
            hole.projection_edge = proj
            hole.ID = proj.ID




    def match_holes_and_projections(self):
        """
        1. Generate randomly 4 points from model_centers_selected
        2. Permutate in the model_centers and find the corresponding model_centers_selected that matches the chosen model_centers_selected
        3. For each choice, solve the PnP problem and evaluate the result using calculate_score, then select the points (and R,t corresponding) with the best score
        """
        
        best_score = np.inf
        best_distances = None

        centers_ellipses = self.get_ellipses_centers(self.image.ellipses)     # imagepoints
        centers_holes_flangeRF = self.get_holes_centers(self.holes, "flange") # objpoints 
       
        # 1. Divide the imagepoints in 2 sets (selected - remaining) 
        selected_indices_random = np.array([0, 1, 2, 3])
        centers_ellipses_selected, centers_ellipses_remaining = self.split_set(centers_ellipses, selected_indices_random)

        # 2. Iteratively search for the corresponding objectpoints  
        indices = list(range(len(centers_holes_flangeRF)))
        for selected_indices in itertools.permutations(indices, 4):
            selected_indices = np.array(selected_indices)
            remaining_indices = [i for i in indices if i not in selected_indices]

            # Randomly divide the objpoints set in 2 sets (selected - remaining)
            model_centers_selected = np.array([centers_holes_flangeRF[i] for i in selected_indices])
            model_centers_remaining = np.array([centers_holes_flangeRF[i] for i in remaining_indices])

            # Solve initial PnP
            self.success, rvec, self.tvec = cv2.solvePnP(model_centers_selected, centers_ellipses_selected, self.camera_params.CAMERA_MATRIX, None, flags=cv2.SOLVEPNP_ITERATIVE)

            if not self.success: continue
            self.R, _ = cv2.Rodrigues(rvec)

            # Project all points and evaluate this choice 
            threshold =  self.pose_estimation_params.TRESHOLD_BF_REPROJECTION_ERROR
            self.success, matches = self.check_PnP(self.R, self.tvec, model_centers_remaining, centers_ellipses_remaining, threshold)

            score = self.calculate_score(matches)
            if best_score > score : 
                best_score = score
                best_distances = matches[:, 2]

            if self.success: 
                # found valid solution
                self.sort_holes_and_projections(matches, selected_indices, remaining_indices, selected_indices_random)
                if self.debug_params.MORE_DETAILS: print(f"-[MORE DETAILS] Brute force initialization best score average = {score:.3}[pixels]")
                if self.debug_params.MORE_DETAILS: print(f"-[MORE DETAILS] Brute force initialization best distances = {matches[:, 2]}[pixels]")
                self.T = SE3.Rt(self.R, self.tvec)
                return True, self.holes
            
        if self.debug_params.MORE_DETAILS: 
            print(f"-[FAIL] Brute force initialization best score average = {best_score:.2}[pixels]")
            print(f"-[FAIL] Brute force initialization best distances = {best_distances}[pixels]")
        return False, None, None # If no valid solution is found

        