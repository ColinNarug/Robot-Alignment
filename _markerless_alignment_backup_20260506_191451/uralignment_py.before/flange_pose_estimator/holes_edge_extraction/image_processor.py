import cv2
import numpy as np
from scipy.signal import find_peaks



class ImageProcessor():

    def __init__(self, params: object, timer: object) -> None:
        self.image_processing_params = params.image_processing_params
        self.timer = timer



    @staticmethod
    def compute_histogram(gray_image: np.ndarray) -> np.ndarray:
        """
        Compute histogram [0..255] of a grayscale image
        """
        hist = cv2.calcHist([gray_image], [0], None, [256], [0, 256])
        return hist.flatten()



    @staticmethod
    def moving_average(hist: np.ndarray, window_size: int) -> np.ndarray:
        """
        Apply a moving average to a histogram
        """
        kernel = np.ones(window_size, dtype=np.float32) / window_size
        smoothed = np.convolve(hist, kernel, mode='same')
        return smoothed



    @staticmethod
    def find_two_peaks(histo_final: np.ndarray, min_distance: int) -> tuple[tuple[int, float], tuple[int, float]]:
        """
        Find two major peaks of the histogram
        """
        peaks, _ = find_peaks(histo_final,distance=min_distance)  # Find all peaks
        peak_values = histo_final[peaks]
        sorted_peaks = sorted(zip(peaks, peak_values), key=lambda x: x[1], reverse=True)
        if len(sorted_peaks) < 2:
            return (0, 0), (255, 0)
        p1, p2 = sorted_peaks[:2]
        if p1[0] > p2[0]:
            p1, p2 = p2, p1
        return p1, p2



    @staticmethod
    def compute_gain_offset(real1: int, real2: int, desired1: int, desired2: int) -> tuple[float, float]:
        """
        Compute gain and offset given real peaks and desired peaks
        """
        if real2 != real1:
            gain = (desired2 - desired1) / float(real2 - real1)
            offset = desired1 - gain * real1
        else:
            gain, offset = 1.0, 0.0
        return gain, offset



    @staticmethod
    def apply_gain_offset(img_gray: np.ndarray, gain: float, offset: float) -> np.ndarray:
        """"
        Apply linear gain/offset to a grayscale image with clip [0..255]
        """
        temp = img_gray.astype(np.float32) * gain + offset
        return np.clip(temp, 0, 255).astype(np.uint8)



    def adjust_gain_and_offset(self, ROI: np.ndarray) -> np.ndarray:
        """
        Adjust brightness and contrast of the image by calculating
        the intensity histogram and moving the 2 peaks in a desired position
        """
        min_distance = abs(self.image_processing_params.PEAK2 - self.image_processing_params.PEAK1)//4
        w_size = 15 # smoothing window size
        
        hist_smoothed = self.moving_average(self.compute_histogram(ROI), w_size)
        p1,p2 = self.find_two_peaks(hist_smoothed, min_distance)

        # Compute gain/offset to shift real peaks to desired peaks
        gain, offset = self.compute_gain_offset(p1[0] if len(p1) >= 1 else 0,
                                                p2[0] if len(p2) >= 2 else 255,
                                                self.image_processing_params.PEAK1, 
                                                self.image_processing_params.PEAK2)
        # Apply gain/offset
        ROI_adjusted = self.apply_gain_offset(ROI, gain, offset)
        return ROI_adjusted



    def get_edge(self, ROI: np.ndarray) -> np.ndarray:
        """
        Extracts the hole edge contour points from a given region of interest (ROI) using 
        a combination of bilateral filtering, intensity adjustment, and canny edge detection.
        """
        # Convert to b/w
        ROI = cv2.cvtColor(ROI, cv2.COLOR_BGR2GRAY)
        # Apply bilateral filter
        ROI_blurred = cv2.bilateralFilter(ROI, 
                                          self.image_processing_params.BILATERAL_DIAMETER * 2 + 1, 
                                          self.image_processing_params.SIGMA_COLOR * 2, 
                                          self.image_processing_params.SIGMA_SPACE * 2)
        # Adjust brightness and gain
        ROI_adjusted = self.adjust_gain_and_offset(ROI_blurred)
        # extraction
        ROI_canny = cv2.Canny(ROI_adjusted, 
                              self.image_processing_params.CANNY_THRESHOLD_LOW, 
                              self.image_processing_params.CANNY_THRESHOLD_HIGH)
        return ROI_canny 