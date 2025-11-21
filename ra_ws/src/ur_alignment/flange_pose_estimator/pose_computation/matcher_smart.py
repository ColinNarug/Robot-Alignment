import numpy as np
from typing import Optional

from ur_alignment.flange_pose_estimator.utility_classes import Ellipse, Hole
from .reconstructor_projected_circle import ReconstructorProjectedCircle
from .pose_computation_core import PoseEstimatorCore



class MatcherSmart(PoseEstimatorCore):

    """
    Match the holes with the corresponding projection using a "smart" method --> match_holes_and_projections
    """

    def __init__(self, pose_estimator: PoseEstimatorCore, holes: list[Hole]) -> None:

        super().__init__(image = pose_estimator.image, 
                         T_past = pose_estimator.T_past, 
                         params = pose_estimator.params,
                         holes = holes
                        )
        # Parameters
        self.crown_radius = self.flange_model_params.CROWNS[0].CROWN_RADIUS
        self.angle_step_model = self.flange_model_params.CROWNS[0].ANGLE_STEP
            

    def _get_crown_projection(self) -> Ellipse:
        """
        Fit an ellipse passing for the centers of the holes projections (ellipses)
        """
        ellipses = self.image.ellipses
        ellipses_centers = np.array([[ellipse.cx, ellipse.cy] for ellipse in ellipses])
        crown_projection = Ellipse()
        crown_projection.set_coefficients_from_points(ellipses_centers)
        
        return crown_projection



    def _sort_holes_projections(self, crown_projection: Ellipse) -> tuple[np.ndarray, np.ndarray]:
        """
        The holes projections can be ordered by being localized in polar coordinates. 
        Consider the ellipse created fitting the centers of the holes projections contours (it's the holes crown projection)
        Consider a polar RF (distance, angle), that has as center the center of the ellipse above. 
        The centers are located in this polar RF anf then ordered by "angle"
        """
        
        ellipses_centers = np.array([[ellipse.cx, ellipse.cy] for ellipse in self.image.ellipses])
        cx_crown, cy_crown, a, b, angle = crown_projection.get_parameters()

        # calculate the theta coordinates in the polar RF
        dx = ellipses_centers[:, 0] - cx_crown
        dy = ellipses_centers[:, 1] - cy_crown
        thetas = np.arctan2(dy, dx) % (2 * np.pi)  # return result in [0, 2*pi]

        # Order ellipses based on theta
        sort_idx = np.argsort(thetas)
        ellipses_sorted = np.array([self.image.ellipses[i] for i in sort_idx])
        ellipses_centers_sorted = np.array([[ellipse.cx, ellipse.cy] for ellipse in ellipses_sorted])

        return ellipses_sorted, ellipses_centers_sorted
    


    def backproject_imagepoints_to_plane(self, imgpoints: np.ndarray, camera_matrix: np.ndarray, plane_center: np.ndarray, plane_normal: np.ndarray) -> np.ndarray: 
        """
        For each imgpoints provided, calculates the intersection of:
        - The radius of all the possible 3D points that generated an imgpoints 
        - The planed defined by (plane_center, plane_normal).
        """
        camera_matrix_inv = np.linalg.inv(camera_matrix)
        points_3d = []

        for imgpoint in imgpoints:
            imgpoint_homog = np.array([imgpoint[0], imgpoint[1], 1.0]) # homogeneous

            # radius
            radius = camera_matrix_inv @ imgpoint_homog
            radius_normalized = radius / np.linalg.norm(radius)

            # plane intersection
            intrinsic_distance = np.dot(plane_center, plane_normal) / np.dot(radius_normalized, plane_normal) 
            point_3d = intrinsic_distance * radius_normalized
            points_3d.append(point_3d)

        return np.array(points_3d)
    

    @staticmethod
    def compute_relative_angles_3D(points_3D: np.ndarray, center: np.ndarray, normal: np.ndarray) -> np.ndarray:
        """
        Given 3D ordered points lying on the a plane, compute the polar angles in a local reference frame
        defined by the plane's normal. Then, based on the absolute angles, compute the angular 
        differences between adjacent points, including the wrap-around between the last and the first point.
        """
        # Define local RF, starting from "normal" vercor
        arbitrary = np.array([1, 0, 0], dtype=float)
        if np.abs(np.dot(normal, arbitrary)) > 0.99: arbitrary = np.array([0, 1, 0], dtype=float)
        u_axis = np.cross(normal, arbitrary)
        u_axis /= np.linalg.norm(u_axis)
        v_axis = np.cross(normal, u_axis)
        v_axis /= np.linalg.norm(v_axis)
        
        # Compute angles
        angles = []
        for point_3D in points_3D:
            distance = point_3D - center
            x_comp = np.dot(distance, u_axis)
            y_comp = np.dot(distance, v_axis)
            ang = np.arctan2(y_comp, x_comp)
            angles.append(ang)
        angles = np.unwrap(np.array(angles))

        # Compute angles differences
        relative_angles = np.diff(angles)
        # Add last-to-first angle 
        last_diff = (angles[0] + 2*np.pi) - angles[-1]
        relative_angles = np.append(relative_angles, last_diff)
        
        return relative_angles
    


    def _get_relative_angles_candidates(
        self, crowns_circles_candidates: list[Hole], ellipses_centers: np.ndarray) -> np.ndarray:
        """
        Compute relative angles for each crown candidate from backprojected points.
        """
        relative_angles_candidates = np.array([])
        for crown_circle_candidate in crowns_circles_candidates:

            candidate_center = crown_circle_candidate["camera"].center
            candidate_normal = crown_circle_candidate["camera"].normal

            # Backproject imagepoints the 2 candidates crown planes
            pts_3d_candidate_ordered = self.backproject_imagepoints_to_plane(ellipses_centers, self.camera_params.CAMERA_MATRIX,
                                                                             candidate_center, candidate_normal)
            # Relative angles, of the holes, in the crown planes RF
            relative_angles = self.compute_relative_angles_3D(pts_3d_candidate_ordered, candidate_center, candidate_normal)
            relative_angles_candidates = np.vstack([relative_angles_candidates, relative_angles]) if relative_angles_candidates.size else np.array([relative_angles])

        return relative_angles_candidates
        
    

    def _select_best_relative_angles_set(self, relative_angles_candidates: np.ndarray) -> int:
        """
        Find the relative angles that best fit with the model, in terms of the 
        relative center angle between every consecutive holes.
        """
        min_error_candidate = np.inf
        idx_winner = None
        for idx, relative_angles_candidate in enumerate(relative_angles_candidates):
            # error from the closest multiple of angle_step_model
            errors_array_candidate = relative_angles_candidate - np.round(relative_angles_candidate / self.angle_step_model) * self.angle_step_model
            error_candidate = np.linalg.norm(errors_array_candidate)

            if error_candidate < min_error_candidate:
                min_error_candidate = error_candidate
                idx_winner = idx
        
        return idx_winner
    


    def _sort_holes(
        self, relative_angles_candidates: np.ndarray, index_winner: int, ellipses: np.ndarray) -> list[Hole]:
        """
        Sort the holes in the list so they match the corresponding 
        projection ellipses. Also return the holes that were skipped.
        """

        relative_angles = relative_angles_candidates[index_winner]
        skip_step_array = np.round(relative_angles / self.angle_step_model, 0) - 1

        current_hole_index = 0
        tracked_indices = []
        untracked_indices = []

        for skip_step in skip_step_array:
            skip = int(max(skip_step, 0))

            # Indices of the skipped holes
            for i in range(1, skip + 1):
                skipped_index = current_hole_index + i
                if skipped_index < len(self.holes):
                    untracked_indices.append(skipped_index)

            if current_hole_index < len(self.holes): tracked_indices.append(current_hole_index)

            current_hole_index += 1 + skip

        holes_tracked = self._build_holes_from_indices(holes_source = self.holes, indices = tracked_indices, projection_edges = ellipses, is_tracked = True)
        holes_not_tracked = self._build_holes_from_indices(holes_source = self.holes, indices = untracked_indices, projection_edges = None, is_tracked = False)
        holes = holes_tracked + holes_not_tracked
        return holes



    def _build_holes_from_indices(
        self, holes_source: list[Hole], indices: list[int], projection_edges: Optional[list[Ellipse]], is_tracked: bool) -> list[Hole]:       
        """
        Build Hole objects from a list of indices.
        If is_tracked is True, assign projection_edge and ID from projection_edges.
        Otherwise, set them to None.
        """
        holes_result = []

        for i, index in enumerate(indices):
            hole = Hole()
            hole.ID_model = holes_source[index].ID_model
            hole.radius = holes_source[index].radius
            hole["flange"].center = holes_source[index]["flange"].center
            hole["flange"].normal = holes_source[index]["flange"].normal

            if is_tracked:
                hole.projection_edge = projection_edges[i]
                hole.ID = projection_edges[i].ID
            else:
                hole.projection_edge = None
                hole.ID = None

            holes_result.append(hole)

        return holes_result




    def match_holes_and_projections(self) -> list[Hole]:
        """
		Main method to match hole projections to model using polar geometry and angular comparison.
		"""

        # Get the ellipse produced fitting the holes centers projection (the holes crown projection basically)
        crown_projection = self._get_crown_projection() 

        # Sort the holes (and their centers) based on their position on the image
        ellipses, ellipses_centers = self._sort_holes_projections(crown_projection)

        # Get 2 3D circles candidates for the holes crown (immaginary circle passing through centers of the holes)
        projected_cicle_reconstructor = ReconstructorProjectedCircle()
        crowns_circles_candidates = projected_cicle_reconstructor.get_circles(crown_projection, self.camera_params.CAMERA_MATRIX, self.crown_radius)

        # Get the 2 sets of relative centers angles, belonging to the two candidaes crowns
        relative_angles_candidates = self._get_relative_angles_candidates(crowns_circles_candidates, ellipses_centers)
        
        # Get the index [0,1] of the correct set of relative centers angles (so the correct crown)
        idx_winner = self._select_best_relative_angles_set(relative_angles_candidates)

        # Sort the holes in the flange Reference Frame
        holes = self._sort_holes(relative_angles_candidates, idx_winner, ellipses)

        return holes

