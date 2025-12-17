import numpy as np
from dataclasses import dataclass
from typing import List, Dict



"""
Series of dataclasses to store the parameters defined in the "config.yaml" file
The attributes documentation is equivalent to the one in "config.yaml"
"""

@dataclass
class DebugParams:
    DETAILS: bool
    MORE_DETAILS: bool
    DRAW_RESULTS: bool


@dataclass
class CameraParams:
    FX: float
    FY: float
    UX: float
    UY: float
    DISTORTION_COEFFS: np.ndarray
    CAMERA_MATRIX: np.ndarray


@dataclass
class DetectionParams:
    MODEL_PATH: str
    MIN_CONFIDENCE_HOLE: float
    MIN_CONFIDENCE_FLANGE: float
    BB_INCREASE_RATIO: float


@dataclass
class ImageProcessingParams:
    BILATERAL_DIAMETER: int
    SIGMA_COLOR: float
    SIGMA_SPACE: float
    PEAK1: int
    PEAK2: int
    CANNY_THRESHOLD_LOW: int
    CANNY_THRESHOLD_HIGH: int


@dataclass
class EllipseFittingParams:
    MIN_ELLIPSE_AREA_RATIO: float
    DISTANCE_AREA_INCREASE: float
    DISTANCE_AREA_DECREASE: float
    MAX_NUM_GOOD_ITERATIONS: int
    MAX_NUM_ITERATIONS: int
    MIN_INLIERS_OVER_ELLIPSE_PERIMETER_RATIO: float
    MIN_INLIERS_OVER_TOTAL_RATIO: float
    MAX_PERCENTAGE_INLIERS: float
    MIN_NUM_PIXELS_CANNY: int


@dataclass
class EllipseExtractionParams:
    MODEL_PATH: str
    MIN_CONFIDENCE_HOLE: float
    ROI_SIZE: List[int]
    ENCODER_NAME: str
    NUM_IN_CHANNELS: int


@dataclass
class PoseEstimationParams:
    TRESHOLD_TRACKING_REPROJECTION_ERROR: float
    TRESHOLD_BF_REPROJECTION_ERROR: float
    TRESHOLD_SCALAR: float
    TRESHOLD_REFINED_REPROJECTION_ERROR: float
    TRESHOLD_COARSE_REPROJECTION_ERROR: float
    TRESHOLD_HOMOGRAPHY_DISTANCE: float


@dataclass
class FlangeCrownParams:
    from .hole import Hole
    CROWN_RADIUS: float
    HOLE_RADIUS: float
    HEIGHT: float
    NUMBER_OF_HOLES: int
    SKIP_INDICES: List[int]
    ANGLE_OFFSET: float
    HOLES: List[Hole]
    ANGLE_STEP: float


@dataclass
class FlangeModelParams:
    MINIMUM_NUM_ACTIVE_HOLES: int
    TOTAL_NUM_HOLES: int
    TOTAL_NUM_CROWNS: int
    NUM_UNIQUE_HEIGHTS: int
    CROWNS: List[FlangeCrownParams]
 

@dataclass
class Params:
    flange_model_params: FlangeModelParams
    pose_estimation_params: PoseEstimationParams
    ellipse_fitting_params: EllipseFittingParams
    ellipse_extraction_params: EllipseExtractionParams
    image_processing_params: ImageProcessingParams
    detection_params: DetectionParams
    camera_params: CameraParams
    debug_params: DebugParams
    method_choice: int = 1  
 


def get_flange_data(config: Dict) -> FlangeModelParams:
	"""
	Extract flange geometry parameters from config and return a FlangeModelParams object.
	Estrae i parametri geometrici della flangia dal dizionario di configurazione.
	"""
	from .hole import Hole  

	FLANGE_PARAMS = config.get("FLANGE_PARAMS")
	CROWNS = FLANGE_PARAMS["CROWNS"]

	crowns = []
	total_num_crowns = 0
	total_num_holes = 0
	unique_heights = set()

	for CROWN in CROWNS:
		total_num_crowns += 1

		CROWN_RADIUS = CROWN["CROWN_RADIUS"]
		HOLE_RADIUS = CROWN["HOLE_RADIUS"]
		HEIGHT = CROWN["HEIGHT"]
		NUMBER_OF_HOLES = CROWN["NUMBER_OF_HOLES"]
		SKIP_INDICES = CROWN["SKIP_INDICES"]
		ANGLE_OFFSET = CROWN["ANGLE_OFFSET"]

		unique_heights.add(HEIGHT)
		angle_step_model = 2 * np.pi / NUMBER_OF_HOLES

		holes = []
		for idx in range(NUMBER_OF_HOLES):
			if idx in SKIP_INDICES:
				continue

			angle = idx * angle_step_model + ANGLE_OFFSET
			x = CROWN_RADIUS * np.cos(angle)
			y = CROWN_RADIUS * np.sin(angle)

			hole = Hole()
			hole["flange"].center = np.array([x, y, HEIGHT])
			hole["flange"].normal = np.array([0, 0, 1])
			hole.radius = HOLE_RADIUS
			hole.ID_model = idx

			holes.append(hole)
			total_num_holes += 1

		crowns.append(
			FlangeCrownParams(**CROWN, HOLES=holes, ANGLE_STEP=angle_step_model)
		)

	MINIMUM_NUM_ACTIVE_HOLES = FLANGE_PARAMS["MINIMUM_NUM_ACTIVE_HOLES"]
	num_unique_heights = len(unique_heights)

	flange_model_params = FlangeModelParams(
		MINIMUM_NUM_ACTIVE_HOLES,
		total_num_holes,
		total_num_crowns,
		num_unique_heights,
		crowns
	)

	return flange_model_params



def parse_config(config: dict) -> Params:
    """
    Parses the raw config dict into structured dataclasses used throughout the pipeline
    """
    # Debug parameters
    debug_params = config["DEBUG_PARAMS"]
    debug_params = DebugParams(**debug_params)

    # Camera parameters
    cam = config["CAMERA_PARAMETERS"]
    camera_params = CameraParams(
                    FX = cam["FX"],
                    FY = cam["FY"],
                    UX = cam["UX"],
                    UY = cam["UY"],
                    DISTORTION_COEFFS = np.array(cam["DISTORTION_COEFFS"]),
                    CAMERA_MATRIX = np.array([[cam["FX"], 0,         cam["UX"]], 
                                              [0,         cam["FY"], cam["UY"]], 
                                              [0,         0,                 1]]))

    # Flange model parameters
    flange_model_params = get_flange_data(config)


    # Detection parameters
    det = config["DETECTION_PARAMS"]
    detection_params = DetectionParams(**det)


    # Image processing parameters
    proc = config["CONTOUR_PARAMS"]
    image_processing_params = ImageProcessingParams(**proc)


    # Ellipse fitting (RANSAC)
    ellipse_fit = config["RANSAC_PARAMS"]
    ellipse_fitting_params = EllipseFittingParams(**ellipse_fit)
    
    
    # Ellipse extraction (segmentation)
    ellipse_seg = config["SEGMENTATION_PARAMS"]
    ellipse_extraction_params = EllipseExtractionParams(**ellipse_seg)


    # Pose estimation
    pose_est = config["RANSAC_ITERATIVE_PNP_PARAMS"]
    pose_estimation_params = PoseEstimationParams(**pose_est)


    params = Params(debug_params = debug_params,
                    camera_params = camera_params,
                    flange_model_params = flange_model_params,
                    detection_params = detection_params,
                    image_processing_params = image_processing_params,
                    ellipse_fitting_params = ellipse_fitting_params,
                    ellipse_extraction_params = ellipse_extraction_params,
                    pose_estimation_params = pose_estimation_params,
                    method_choice = config["METHOD_CHOICE"]          
    )
    
    return params

