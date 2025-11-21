#!/usr/bin/env python3
"""Run YOLO inference on a folder of PNG frames, with optional preview and save."""

import os, re, time, cv2, numpy as np
from ultralytics import YOLO
import supervision as sv
from paths import Paths  # scripts/paths.py
from pathlib import Path



# ============================== #
# ======= TUNABLE PARAMS ======= #
# ============================== #
INPUT_SUBDIR  = "original/cavity_images"                
MODEL_WEIGHTS = "runs/train/weights/best.pt"
OUTPUT_SUBDIR = "runs/output/video_detection_recorded"

SHOW_WINDOW    = True     # show preview window
SAVE_OUTPUT    = False     # save annotated PNGs
SHOW_FLANGE    = False    # draw best flange + its holes; if False, draw all holes
FRAME_RATE     = 30       # max FPS for processing loop

CONF_FLANGE    = 0.7      # min confidence for flange detection
CONF_HOLE      = 0.7      # min confidence for hole detection

# Class IDs used by the trained model
FLANGE_CLASS_ID = 1
HOLE_CLASS_ID   = 0

# BGR drawing colors
COLORS = {
    FLANGE_CLASS_ID: (50, 255, 0),
    HOLE_CLASS_ID:   (80, 150, 240),
}
# ============================== #

ROOT_DIR = Path(__file__).resolve().parents[1]  # repo root
INPUT_PATH  = ROOT_DIR / INPUT_SUBDIR
MODEL_PATH  = ROOT_DIR / MODEL_WEIGHTS
OUTPUT_PATH = ROOT_DIR / OUTPUT_SUBDIR


def natural_key(s: str):
    """Numeric-friendly sort key: frame_2.png < frame_10.png."""
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r"(\d+)", s)]


class Detector:
    """Thin wrapper around Ultralytics YOLO + conversion to supervision.Detections."""

    def __init__(self, model_path: str) -> None:
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f"Missing model: {model_path}")
        self.model = YOLO(model_path)
        # print(f"[INFO] classes: {self.model.names}")

    def infer(self, frame: np.ndarray) -> sv.Detections | None:
        res = self.model(frame)
        if not res or len(res[0].boxes) == 0:
            return None
        return sv.Detections.from_ultralytics(res[0])

    def _best_flange(self, dets: sv.Detections):
        mask = (dets.class_id == FLANGE_CLASS_ID) & (dets.confidence > CONF_FLANGE)
        flange = dets[mask]
        if len(flange) == 0:
            return None, None
        idx = int(np.argmax(flange.confidence))
        return flange.xyxy[idx], float(flange.confidence[idx])

    @staticmethod
    def _inside(box, pt) -> bool:
        x1, y1, x2, y2 = box
        x, y = pt
        return x1 <= x <= x2 and y1 <= y <= y2

    def draw(self, frame: np.ndarray, dets: sv.Detections | None, show_flange: bool) -> np.ndarray:
        """If show_flange: draw best flange and holes inside. Else: draw all holes."""
        if dets is None:
            return frame

        if show_flange:
            flange_box, flange_conf = self._best_flange(dets)
            if flange_box is not None:
                x1, y1, x2, y2 = map(int, flange_box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), COLORS[FLANGE_CLASS_ID], 2)
                cv2.putText(frame, f"Flange {flange_conf:.2f}", (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[FLANGE_CLASS_ID], 2)

                hole_mask = (dets.class_id == HOLE_CLASS_ID) & (dets.confidence > CONF_HOLE)
                hole_dets = dets[hole_mask]
                for j, hbox in enumerate(hole_dets.xyxy):
                    hx1, hy1, hx2, hy2 = map(int, hbox)
                    cx, cy = (hx1 + hx2) / 2.0, (hy1 + hy2) / 2.0
                    if self._inside((x1, y1, x2, y2), (cx, cy)):
                        cv2.rectangle(frame, (hx1, hy1), (hx2, hy2), COLORS[HOLE_CLASS_ID], 2)
                        cv2.putText(frame, f"Hole {hole_dets.confidence[j]:.2f}", (hx1, max(0, hy1 - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[HOLE_CLASS_ID], 2)
        else:
            hole_mask = (dets.class_id == HOLE_CLASS_ID) & (dets.confidence > CONF_HOLE)
            hole_dets = dets[hole_mask]
            for j, hbox in enumerate(hole_dets.xyxy):
                hx1, hy1, hx2, hy2 = map(int, hbox)
                cv2.rectangle(frame, (hx1, hy1), (hx2, hy2), COLORS[HOLE_CLASS_ID], 2)
                cv2.putText(frame, f"Hole {hole_dets.confidence[j]:.2f}", (hx1, max(0, hy1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[HOLE_CLASS_ID], 2)

        return frame


class FolderProcessor:
    """Iterate frames from a folder, run inference, draw, show/save, and cap FPS."""

    def __init__(self, input_dir: str, output_dir: str, detector: Detector,
                 show: bool, save: bool, show_flange: bool, max_fps: int) -> None:
        self.input_dir = input_dir
        self.output_dir = output_dir
        self.detector = detector
        self.show = show
        self.save = save
        self.show_flange = show_flange
        self.dt_target = 1.0 / max(1, int(max_fps))

    # -- listing section --
    def _list_frames(self) -> list[str]:
        if not os.path.isdir(self.input_dir):
            raise FileNotFoundError(f"Missing input folder: {self.input_dir}")
        files = [f for f in os.listdir(self.input_dir) if f.lower().endswith(".png")]
        files.sort(key=natural_key)
        return files

    # -- main loop section --
    def process(self) -> None:
        files = self._list_frames()
        total = len(files)
        if total == 0:
            print("[WARN] Empty folder.")
            return

        if self.save:
            os.makedirs(self.output_dir, exist_ok=True)

        win = "Detections"
        if self.show:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)

        for i, fname in enumerate(files, 1):
            t0 = time.perf_counter()

            path = os.path.join(self.input_dir, fname)
            frame = cv2.imread(path)
            if frame is None:
                continue

            dets = self.detector.infer(frame)
            out = self.detector.draw(frame, dets, self.show_flange)

            if self.save:
                cv2.imwrite(os.path.join(self.output_dir, fname), out)

            if self.show:
                cv2.imshow(win, out)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            # FPS cap
            elapsed = time.perf_counter() - t0
            if elapsed < self.dt_target:
                time.sleep(self.dt_target - elapsed)

            if i % 50 == 0:
                print(f"[INFO] {i}/{total}")

        if self.show:
            cv2.destroyAllWindows()
        print(f"[DONE] {total} frame(s). Output: {self.output_dir if self.save else 'none'}")


def main() -> None:
    print("[DEBUG] Initializing paths...", flush=True)
    paths = Paths.from_repo_root(weights_name=MODEL_WEIGHTS)

    input_dir  = os.path.join(paths.root, INPUT_SUBDIR)
    model_path = paths.weights_path
    output_dir = os.path.join(paths.root, OUTPUT_SUBDIR)

    print(f"[DEBUG] Root directory     : {paths.root}", flush=True)
    print(f"[DEBUG] Input folder       : {input_dir}", flush=True)
    print(f"[DEBUG] Model weights file : {model_path}", flush=True)
    print(f"[DEBUG] Output folder      : {output_dir}", flush=True)
    print(f"[DEBUG] Show window? {SHOW_WINDOW}, Save output? {SAVE_OUTPUT}", flush=True)

    if not os.path.exists(model_path):
        print(f"[ERROR] Model file not found at {model_path}", flush=True)
        return

    if not os.path.isdir(input_dir):
        print(f"[ERROR] Input folder not found at {input_dir}", flush=True)
        return

    det = Detector(model_path)
    print("[DEBUG] Detector initialized successfully.", flush=True)

    processor = FolderProcessor(
        input_dir=input_dir,
        output_dir=output_dir,
        detector=det,
        show=SHOW_WINDOW,
        save=SAVE_OUTPUT,
        show_flange=SHOW_FLANGE,
        max_fps=FRAME_RATE,
    )
    print("[DEBUG] Starting folder processing...", flush=True)
    processor.process()
    print("[DEBUG] Processing finished.", flush=True)


if __name__ == "__main__":
    main()
