import numpy as np
import supervision as sv
from ultralytics import YOLO
from supervision.tracker.byte_tracker.core import ByteTrack
from dataclasses import dataclass, field
from typing import Tuple, List, Any

from ur_alignment.flange_pose_estimator.utility_classes import Params, Timer


# --------------------------------------------------------------------------- #
# Data classes
# --------------------------------------------------------------------------- #
@dataclass
class ObjectDetected:
    """
    Used to store an object detected by the `.detect()` method of the `Detector`
    class (either a flange or a hole).
    """
    ID: int                                      # Unique and stable SORT ID
    ROI_coordinates: Tuple[int, int, int, int]   # (x1, y1, x2, y2) in original img
    ROI: Any                                     # Image crop of the detection
    confidence: float                            # Detection confidence score
    contained_objects: List['ObjectDetected'] = field(default_factory=list)

    def add_contained_object(self, contained_object: 'ObjectDetected') -> None:
        """Attach a nested detection (e.g. a hole inside a flange)."""
        self.contained_objects.append(contained_object)

    def __repr__(self):
        return f"ID = {self.ID} -> ROI coordinates = {self.ROI_coordinates}\n"



# --------------------------------------------------------------------------- #

class Detector:
    """
    Flange‑and‑hole detector & tracker.

    Call `detect(frame)` each frame. The class keeps two independent SORT
    trackers – one for flanges, one for holes – to ensure consistent IDs across
    time even if detections are temporarily lost.
    """

    def __init__(self, params: Params, timer: Timer) -> None:
        # ------------------------------------------------------------------- #
        # Model & configuration
        # ------------------------------------------------------------------- #
        self.detection_params      = params.detection_params      # thresholds, bb ratio…
        self.flange_model_params   = params.flange_model_params   # expected #holes, etc.
        self.model                 = YOLO(self.detection_params.MODEL_PATH,
                                          verbose=False)

        # ------------------------------------------------------------------- #
        # Trackers
        # ------------------------------------------------------------------- #
        self.flange_tracker = ByteTrack(minimum_consecutive_frames = 3)
        self.hole_tracker   = ByteTrack(minimum_consecutive_frames = 3)

        # ------------------------------------------------------------------- #
        # Per‑frame storage
        # ------------------------------------------------------------------- #
        self.flanges_detected: List[ObjectDetected] = []
        self.holes_detected  : List[ObjectDetected] = []

        # ------------------------------------------------------------------- #
        # Debug
        # ------------------------------------------------------------------- #
        self.DETAILS = params.debug_params.DETAILS
        self.timer   = timer


    # ----------------------------------------------------------------------- #
    # Utility
    # ----------------------------------------------------------------------- #
    def expand_ROI(
        self,
        x1: float, y1: float, x2: float, y2: float,
        img_w: int, img_h: int
    ) -> Tuple[int, int, int, int]:
        """
        Expand the bounding box by BB_INCREASE_RATIO, then enforce a square ROI.
        The shorter side is extended symmetrically (half each side) until it matches
        the longer side. The final square is clipped to image boundaries.
        """
        # 1) Initial expansion by ratio
        w = x2 - x1
        h = y2 - y1
        r = float(self.detection_params.BB_INCREASE_RATIO)
        x1 -= r * w
        y1 -= r * h
        x2 += r * w
        y2 += r * h

        # 2) Determine side lengths
        w = x2 - x1
        h = y2 - y1

        if w > h:
            # Need to extend vertically
            diff = w - h
            y1 -= diff / 2
            y2 += diff / 2
        elif h > w:
            # Need to extend horizontally
            diff = h - w
            x1 -= diff / 2
            x2 += diff / 2

        # 3) Clip to image boundaries
        x1 = int(max(0, round(x1)))
        y1 = int(max(0, round(y1)))
        x2 = int(min(img_w, round(x2)))
        y2 = int(min(img_h, round(y2)))

        # 4) Safety adjustment: ensure still square after clipping
        w = x2 - x1
        h = y2 - y1
        side = min(w, h)
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        x1 = int(round(cx - side / 2))
        y1 = int(round(cy - side / 2))
        x2 = x1 + side
        y2 = y1 + side

        # Final clip to image boundaries
        x1 = max(0, x1); y1 = max(0, y1)
        x2 = min(img_w, x2); y2 = min(img_h, y2)

        return x1, y1, x2, y2




    # ----------------------------------------------------------------------- #
    # Internal helpers
    # ----------------------------------------------------------------------- #
    def _filter_detections(self) -> List[Tuple[int, float, Any]]:
        """
        Keep only flanges (class 1) and holes (class 0) above the respective
        confidence thresholds.  
        Returns a list of `(class_id, confidence, yolo_box)` tuples.
        """
        detections = self.yolo_results[0].boxes
        filtered_detections = []

        for detection in detections:
            class_id = int(detection.cls[0])
            confidence = detection.conf[0].item()

            if class_id == 1 and confidence >= self.detection_params.MIN_CONFIDENCE_FLANGE:
                filtered_detections.append((class_id, confidence, detection))
            elif class_id == 0 and confidence >= self.detection_params.MIN_CONFIDENCE_HOLE:
                filtered_detections.append((class_id, confidence, detection))

        return filtered_detections



    # ----------------------------------------------------------------------- #
    # Utility: find untracked detections
    # ----------------------------------------------------------------------- #
    def _find_untracked(self, original_dets: List[Any], confs: List[float], tracked_boxes: List[List[float]]) -> List[Tuple[List[float], float]]:
        """
        Return list of (box, confidence) for detections that were not assigned
        a tracker ID. Matching is done by exact box coordinates.
        """
        untracked = []
        for det, conf in zip(original_dets, confs):
            box = det.xyxy[0].tolist()
            if box not in tracked_boxes:
                untracked.append((box, conf))
        return untracked


    def _track_objects( self,
                        detections : List[Tuple[int, float, Any]],
                        tracker    : ByteTrack,
                        class_id   : int
                    ) -> List[ObjectDetected]:
        """
        Run SORT on the detections of the requested `class_id` and return a list
        of `ObjectDetected` with stable IDs.  
        Untracked detections are kept with ID = -1.
        """
        # ------------------------------------------------------------------- #
        # Collect boxes for this class only
        # ------------------------------------------------------------------- #
        boxes, confs, original_dets = [], [], []
        for cls, conf, det in detections:
            if cls != class_id:
                continue
            x1, y1, x2, y2 = det.xyxy[0].tolist()
            boxes.append([x1, y1, x2, y2])
            confs.append(conf)
            original_dets.append(det)

        if not boxes:
            return []

        # ------------------------------------------------------------------- #
        # Build detections for the tracker
        # ------------------------------------------------------------------- #
        dets = sv.Detections(
            xyxy      = np.array(boxes),
            confidence= np.array(confs),
            class_id  = np.full(len(boxes), class_id),
        )

        # ------------------------------------------------------------------- #
        # Run tracking
        # ------------------------------------------------------------------- #
        tracked = tracker.update_with_detections(dets)
        img_h, img_w = self.current_frame.shape[:2]
        objects: List[ObjectDetected] = []

        # ------------------------------------------------------------------- #
        # 1) Add tracked detections
        # ------------------------------------------------------------------- #
        for i in range(len(tracked)):
            x1, y1, x2, y2 = tracked.xyxy[i]
            roi_coords     = self.expand_ROI(x1, y1, x2, y2, img_w, img_h)
            roi            = self.current_frame[roi_coords[1]:roi_coords[3],
                                                roi_coords[0]:roi_coords[2]]
            objects.append(
                ObjectDetected(
                    ID              = int(tracked.tracker_id[i]),
                    ROI_coordinates = roi_coords,
                    ROI             = roi,
                    confidence      = float(tracked.confidence[i]),
                )
            )

        # ------------------------------------------------------------------- #
        # 2) Add untracked detections with ID = -1
        # ------------------------------------------------------------------- #
        tracked_boxes = tracked.xyxy.tolist() if len(tracked) > 0 else []
        for box, conf in self._find_untracked(original_dets, confs, tracked_boxes):
            x1, y1, x2, y2 = box
            roi_coords     = self.expand_ROI(x1, y1, x2, y2, img_w, img_h)
            roi            = self.current_frame[roi_coords[1]:roi_coords[3],
                                                roi_coords[0]:roi_coords[2]]
            objects.append(
                ObjectDetected(
                    ID              = -1,
                    ROI_coordinates = roi_coords,
                    ROI             = roi,
                    confidence      = float(conf),
                )
            )

        return objects


    def _associate_holes_to_flanges(self) -> None:
        """
        Attach each hole to the (unique) flange that contains its centre.
        """
        for hole in self.holes_detected:
            x1, y1, x2, y2 = hole.ROI_coordinates
            hole_center = ((x1 + x2) / 2, (y1 + y2) / 2)

            for flange in self.flanges_detected:
                fx1, fy1, fx2, fy2 = flange.ROI_coordinates
                if fx1 <= hole_center[0] <= fx2 and fy1 <= hole_center[1] <= fy2:
                    flange.add_contained_object(hole)
                    break



    def _clear_detections(self) -> None:
        """
        Reset per‑frame detection lists.
        """
        self.flanges_detected.clear()
        self.holes_detected.clear()



    def _check_detection_results(self) -> bool:
        """
        Validate the detection: exactly one flange, and an acceptable number of
        holes inside according to the model parameters.
        """
        num_flanges = len(self.flanges_detected)

        if num_flanges == 0:
            if self.DETAILS:
                print("-[FAIL] No flange detected in the frame.")
            return False

        if num_flanges > 1:
            if self.DETAILS:
                print(f"-[FAIL] Detected {num_flanges} flanges in the frame.")
            return False

        num_holes = len(self.flanges_detected[0].contained_objects)
        if num_holes > self.flange_model_params.TOTAL_NUM_HOLES:
            if self.DETAILS:
                print(f"-[FAIL] Detected {num_holes} holes: more than the holes in the model ({self.flange_model_params.TOTAL_NUM_HOLES}).")
            return False
        
        if num_holes < self.flange_model_params.MINIMUM_NUM_ACTIVE_HOLES:
            if self.DETAILS:
                print(f"-[FAIL] Detected {num_holes} holes: less than the safety lower bound ({self.flange_model_params.MINIMUM_NUM_ACTIVE_HOLES}).")
            return False
        
        # add other checks here ...

        return True



    def detect(self, frame: Any) -> Tuple[bool, List[ObjectDetected]]:
        """
        Run the full detection‑and‑tracking pipeline on `frame`.
        Returns:
            success (bool): True if the result passes all validation checks.
            holes   (List[ObjectDetected]): holes inside the unique flange.
        """
        self._clear_detections()

        self.current_frame = frame

        # 1. Inference ------------------------------------------------------- #
        self.yolo_results = self.model(frame, verbose=False)

        # 2. Filtering & tracking ------------------------------------------- #
        filtered = self._filter_detections()
        self.flanges_detected = self._track_objects(filtered,
                                                    self.flange_tracker, 1)
        self.holes_detected   = self._track_objects(filtered,
                                                    self.hole_tracker,   0)

        # 3. Hierarchy (holes → flange) ------------------------------------- #
        self._associate_holes_to_flanges()

        # 4. Debug info ------------------------------------------------------ #
        if self.DETAILS:
            n_holes = len(self.holes_detected)
            print(f"-[DETAILS] Flanges: {len(self.flanges_detected)}, "
                  f"Holes: {n_holes}")

        # 5. Validation & result -------------------------------------------- #
        success = self._check_detection_results()
        holes = self.flanges_detected[0].contained_objects if success else []

        return success, holes