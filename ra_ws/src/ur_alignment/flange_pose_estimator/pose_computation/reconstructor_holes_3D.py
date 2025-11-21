import numpy as np
from ur_alignment.flange_pose_estimator.utility_classes import Hole

from .pose_computation_core import PoseEstimatorCore
from .reconstructor_projected_circle import ReconstructorProjectedCircle


class ReconstructorHoles3D(PoseEstimatorCore):

    def __init__(self, pose_estimator: PoseEstimatorCore) -> None:

        super().__init__(image = pose_estimator.image, 
                         T = pose_estimator.T,
                         T_past = pose_estimator.T_past, 
                         params = pose_estimator.params,
                         holes = pose_estimator.holes
                        )
        
        self.holes_original = pose_estimator.holes.copy()
        self.success = False



    def get_holes(self) -> list[Hole]:
        """
		Return the list of reconstructed 3D holes. Raise if reconstruction failed.
		"""
        if not self.success:
            raise RuntimeError("Tried to get the reconstructed holes, but the model was not correctly built.")
        return self.holes



    def _pick_valid_hole(self, hole_pair: list[Hole]) -> Hole | None:
        """
        Return the hole whose normal is closest to the z‑axis (R[:,2]) and
        whose scalar product exceeds SCALAR_THRESHOLD.  None if both fail.
        """

        self.R = self.T.R
        z_axis = self.R[:, 2]
        hole0, hole1 = hole_pair

        s0 = np.dot(hole0["camera"].normal, z_axis)
        s1 = np.dot(hole1["camera"].normal, z_axis)

        if self.debug_params.MORE_DETAILS: print(f"-[MORE DETAILS] Scalar products = {s0: .4f}; {s1: .4f}")

        if abs(s0) > abs(s1): return hole0 if abs(s0) > self.pose_estimation_params.TRESHOLD_SCALAR else None
        
        return hole1 if abs(s1) > self.pose_estimation_params.TRESHOLD_SCALAR else None



    def _select_holes_from_candidates(self, holes_pairs: list[list[Hole]]) -> list[Hole]:
        """
        Select valid holes from candidate pairs based on orientation checks.
        """        
        holes = []
        for pair in holes_pairs:

            hole_chosen = self._pick_valid_hole(pair)

            # If it passed the check
            if hole_chosen is not None:
                holes.append(hole_chosen)

        holes = self._restore_holes_without_projection(holes, self.holes_original)
        return holes
    



    def _get_holes_cameraRF_candidates(self, holes: list[Hole]) -> list[list[Hole]]:
        """
        Generate candidate 3D hole reconstructions in camera RF from ellipse projections.
        """

        projected_cicle_reconstructor = ReconstructorProjectedCircle()
        holes_pairs = []

        for hole in holes:

            if hole.projection_edge is None: continue

            candidates = projected_cicle_reconstructor.get_circles(hole.projection_edge, self.camera_params.CAMERA_MATRIX, hole.radius)

            # Also assign the model data
            for candidate in candidates: 
                candidate.ID_model = hole.ID_model
                candidate["flange"].center = hole["flange"].center
                candidate["flange"].normal = hole["flange"].normal

            holes_pairs.append(candidates)

        return holes_pairs



    def _check_result(self, holes: list[Hole]) -> tuple[bool, list[Hole] | None]:
        """
        Check if reconstruction produced a sufficient number of valid holes.
        """
        # Check if the number of holes is sufficient
        success = len(holes) >= self.flange_model_params.MINIMUM_NUM_ACTIVE_HOLES
        if not success:
            if self.debug_params.DETAILS: print(f"-[FAIL] Holes 3D reconstrucion failed: found {len(holes)} correct holes")
            return success, None
        
        return success, holes




    def build_3D_holes_from_view(self) -> bool:
        """
        Main function to reconstruct 3D hole geometry from 2D projections.
        """
        holes_candidates = self._get_holes_cameraRF_candidates(self.holes)

        holes = self._select_holes_from_candidates(holes_candidates)

        self.success, self.holes = self._check_result(holes)

        return self.success
    





