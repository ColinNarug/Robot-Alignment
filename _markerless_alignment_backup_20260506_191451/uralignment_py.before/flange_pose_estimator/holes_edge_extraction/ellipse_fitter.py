import numpy as np
from ur_alignment.flange_pose_estimator.utility_classes import Ellipse


class EllipseFitter:
    def __init__(self, params: object, timer: object) -> None:
        self.params = params
        self.ellipse_fitting_params = params.ellipse_fitting_params
        self.debug_params = params.debug_params
        self.timer = timer
        self._reset_state()



    def _reset_state(self) -> None:
        """
        Reset all internal state related to fitting.
        """       
        self.edge_pixels = None
        self.num_pixels = None
        self.best_fitting_points = None
        self.best_ellipse_ROI = None
        self.best_inliers = None
        self.best_score = None
        self.iteration_good = None
        self.iteration_total = None



    @staticmethod
    def ellipse_ROI_to_full_image(best_ellipse_ROI: Ellipse, ROI_coordinates: tuple[int, int, int, int]) -> Ellipse:
        """
        Map the ellipse calculated in the ROI, in the whole image space
        """
        x1, y1, _, _ = ROI_coordinates
        cx_roi, cy_roi, angle, a, b = best_ellipse_ROI.get_parameters()
        cx = cx_roi + x1
        cy = cy_roi + y1
        ellipse = Ellipse()
        ellipse.ID = best_ellipse_ROI.ID
        ellipse.set_parameters(cx, cy, angle, a, b)
        return ellipse



    def _setup(self, ROI_canny: np.ndarray) -> bool:
        """
        Prepare edge data for RANSAC and validate pixel count.
        """
        self.edge_pixels = np.argwhere(ROI_canny == 255)
        height, width = ROI_canny.shape[:2]
        self.ROI_area = height * width
        self.num_pixels = len(self.edge_pixels)

        if self.num_pixels < self.ellipse_fitting_params.MIN_NUM_PIXELS_CANNY:
            if self.debug_params.MORE_DETAILS:
                print(
                    f"[MORE DETAILS] Could not start the RANSAC, the canny produced {self.num_pixels} pixels"
                    f" (less than {self.ellipse_fitting_params.MIN_NUM_PIXELS_CANNY})")
            return False
        
        return True
 


    def _initialize_ransac(self) -> None:
        """
        Initialize RANSAC state before starting fitting loop.
        """
        self.best_fitting_points = None
        self.best_ellipse_ROI = None
        self.best_inliers = None
        self.best_score = 0
        self.iteration_good = 0
        self.iteration_total = 0



    def _run_ransac(self, ROI_canny: np.ndarray) -> None:
        """
        Run the RANSAC loop to find the best ellipse model.
        """
        while (self.iteration_good < self.ellipse_fitting_params.MAX_NUM_GOOD_ITERATIONS and
               self.iteration_total < self.ellipse_fitting_params.MAX_NUM_ITERATIONS):
            self.iteration_total += 1
            # --------------------------------------------------------------
            # Choose 5 random pixels
            # --------------------------------------------------------------
            selected = np.random.choice(self.num_pixels, 5, replace=False)
            test_pixels = self.edge_pixels[selected]
            test_pixels_swap = test_pixels[:, ::-1] # OpenCV uses a [y, x] convention for pixels

            all_idx = np.arange(self.num_pixels)
            other_idx = np.setdiff1d(all_idx, selected)
            not_selected = self.edge_pixels[other_idx]
            other_swap = not_selected[:, ::-1]

            # --------------------------------------------------------------
            # Fit the ellipse
            # --------------------------------------------------------------
            candidate = Ellipse()
            candidate.set_coefficients_from_points(test_pixels_swap)

            # --------------------------------------------------------------
            # Check
            # --------------------------------------------------------------
            # heuristic: fitting resut
            if candidate.degenerate_points_fitting: continue

            cx, cy, angle, a, b = candidate.get_parameters()

            # heuristic: Ellipse_Area / ROI_Area > ...
            min_area = self.ellipse_fitting_params.MIN_ELLIPSE_AREA_RATIO * self.ROI_area
            if not (candidate.check_if_inside_box(*ROI_canny.shape) and 
                    candidate.get_area() > min_area): 
                continue
            
            # --------------------------------------------------------------
            # Calculate the score
            # --------------------------------------------------------------
            is_inside = candidate.check_points_distance(
                other_swap,
                DISTANCE_AREA_INCREASE=self.ellipse_fitting_params.DISTANCE_AREA_INCREASE,
                DISTANCE_AREA_DECREASE=self.ellipse_fitting_params.DISTANCE_AREA_DECREASE
            )

            score = np.sum(is_inside)

            # --------------------------------------------------------------
            # Check
            # --------------------------------------------------------------
            # heuristic: enough inliers
            if (candidate.calculate_occupied_pixels(ROI_canny.shape) *
                self.ellipse_fitting_params.MIN_INLIERS_OVER_ELLIPSE_PERIMETER_RATIO > score or
                self.num_pixels * self.ellipse_fitting_params.MIN_INLIERS_OVER_TOTAL_RATIO > score):
                continue

            # --------------------------------------------------------------
            # New best result?
            # --------------------------------------------------------------
            self.iteration_good += 1
            if score > self.best_score:
                self.best_score = score
                self.best_ellipse_ROI = candidate
                self.best_fitting_points = test_pixels
                self.best_inliers = np.vstack((not_selected[is_inside], test_pixels))

            # heuristic: if the alforithm got a big percentage of the inliers already, it stops
            if len(self.best_inliers) >= self.num_pixels * self.ellipse_fitting_params.MAX_PERCENTAGE_INLIERS:
                break



    def _fit_final(self, ROI_coordinates: tuple[int, int, int, int], ID: int) -> tuple[bool, Ellipse | None]:
        """
        Fit a final ellipse using the inliers and return it in full image coordinates.
        """
        if self.best_ellipse_ROI is not None:
            # fit a new ellipse using only the inlier points
            ellipse_final = Ellipse()
            ellipse_final.ID = ID
            swap_points = np.vstack((self.best_inliers[:, ::-1],))
            ellipse_final.set_coefficients_from_points(swap_points)
            ellipse_full_img = self.ellipse_ROI_to_full_image(ellipse_final, ROI_coordinates)

            return True, ellipse_full_img
        return False, None



    def get_ellipse(self,
                    ROI_canny: np.ndarray,
                    ROI_coordinates: tuple[int, int, int, int],
                    ID: int
                    ) -> tuple[bool, Ellipse | None]:
        """
        Main method: fit an ellipse on edge points using RANSAC, and return success and ellipse.
        """
        self._reset_state()

        try:
            success = self._setup(ROI_canny)
            if not success: return False, None

            self._initialize_ransac()
            self._run_ransac(ROI_canny)
            return self._fit_final(ROI_coordinates, ID)
        
        finally:
            self._reset_state()
