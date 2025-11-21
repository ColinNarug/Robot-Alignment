import numpy as np
from spatialmath import SE3

from ur_alignment.flange_pose_estimator.pose_computation.pose_computation_core import PoseEstimatorCore
from .pose_initializer import PoseInitializer
from .reconstructor_holes_3D import ReconstructorHoles3D
from .pose_refiner import PoseRefiner
from .pose_evaluator import PoseEvaluator
from ur_alignment.flange_pose_estimator.utility_classes import Timer, Params, Image, Hole


class PoseEstimator(PoseEstimatorCore):

    def __init__(self, params: Params, timer: Timer, T_init: SE3) -> None:

        super().__init__(image = None, 
                         T_past = T_init, 
                         params = params)
        
        self.timer         : Timer      = timer
        self.holes_original: Hole       = self.get_initial_holes_structure() 
        self.holes         : list[Hole] = self.holes_original.copy()

        self.tracking_data : dict[int, np.ndarray] = None
        self.past_tracking_data : dict[int, np.ndarray] = None


    
    def get_initial_holes_structure(self) -> list[Hole]:
        """
        Return an unsorted list of holes in flange RF
        """

        holes = []
        for crown in self.flange_model_params.CROWNS:
            for hole in crown.HOLES:
                holes.append(hole)
        return holes
       


    def get_homography_analysis_results(self) -> np.ndarray:
        """
		Return distances used in homography evaluation.
		"""
        return self.matching_distances
    
    

    def get_image(self) -> Image:
        """
		Return the current Image object.
		"""
        return self.image

    

    def init_pipeline(self, image: Image) -> None:
        """
		Initialize the pose estimation pipeline with the given image.
		"""
        self.image = image
        # Pose estimation success
        self.success = False
        # Results analysis data
        self.matching_distances = None


    
    def _clean_holes_from_cameraRF(self, holes: list[Hole]) -> list[Hole]:
        """
		Remove the 'camera' description from each hole.
		"""
        for hole in holes:
            hole.remove_description("camera")
        return holes
    


    def handle_exit(self) -> None:
        """
		Backup current tracking state and remove camera RF data from holes.
		"""
        self.T_past = self.T
        if self.tracking_data is not None:
            self.past_tracking_data : dict[int, np.ndarray] = self.tracking_data
        else: 
            self.past_tracking_data = None
        self.holes = self._clean_holes_from_cameraRF(self.holes)



    def get_pose(self, image: Image) -> tuple[bool, SE3 | None]:
        """
        Main function to estimate the 6D pose of the flange from the input image.
        """
     
        # Pipeline initialization
        self.init_pipeline(image)
        
        # ------------------------------------------------------------------
        # Pose computation initialization
        # ------------------------------------------------------------------
        pose_initializer = PoseInitializer(self)

        pose_initializer.match_holes_model_and_projections(self.holes, self.past_tracking_data) # holes get a projection associated
        self.success, self.T = pose_initializer.get_initial_pose()
        if not self.success: 
            self.handle_exit()
            return self.success, self.T

        self.holes = pose_initializer.get_holes()

        # ------------------------------------------------------------------
        # Holes reconstruction from camera projection
        # ------------------------------------------------------------------
        holes_3D_reconstructor = ReconstructorHoles3D(self)

        self.success = holes_3D_reconstructor.build_3D_holes_from_view()
        if not self.success: 
            self.handle_exit()
            return self.success, self.T

        self.holes = holes_3D_reconstructor.get_holes()

        # ------------------------------------------------------------------
        # Refined pose estimation
        # ------------------------------------------------------------------
        pose_refiner = PoseRefiner(self)

        self.success, self.holes  = pose_refiner.refine_holes()
        if not self.success: 
            self.handle_exit()
            return self.success, self.T

        self.success, self.T = pose_refiner.get_pose()
        if not self.success: 
            self.handle_exit()
            return self.success, self.T
        
        self.holes = pose_refiner.get_holes()
        self.tracking_data = pose_refiner.get_tracking_data()
        
        # ------------------------------------------------------------------
        # Evaluate the pose estimation
        # ------------------------------------------------------------------
        pose_evaluator = PoseEvaluator(self)

        self.success = pose_evaluator.evaluate_pose()
        if not self.success: 
            self.handle_exit()
            return self.success, self.T
        self.matching_distances = pose_evaluator.get_matching_distances()
        
        if self.debug_params.DRAW_RESULTS: 
            pose_refiner.make_pose_estimation_draw()
            self.image = pose_refiner.get_image()

        self.handle_exit()
        return self.success, self.T
        
