#!/usr/bin/env python3
"""Extract color frames from a RealSense .bag into PNGs with a small preview."""

import os
import cv2
import numpy as np
import pyrealsense2 as rs
from paths import Paths  # scripts/paths.py


# ============================== #
# ======= TUNABLE PARAMS ======= #
# ============================== #
INPUT_BAG_NAME  = "cavity_flange.bag"     # file inside bags/
OUTPUT_SUBDIR   = "cavity_flange"        # folder inside original/
WINDOW_SIZE     = (1920, 1080)            # preview window (w, h)
MAX_FRAMES      = None                   # e.g., 500 or None for all
SHOW_PREVIEW    = True                   # disable to extract without UI
# ============================== #


class BagToPNG:
    """Minimal extractor with preview and graceful shutdown."""

    def __init__(self, input_bag: str, output_dir: str,
                 window_size: tuple[int, int], max_frames: int | None,
                 show_preview: bool) -> None:
        self.input_bag = input_bag
        self.output_dir = output_dir    
        self.win_w, self.win_h = window_size
        self.max_frames = max_frames
        self.show_preview = show_preview
        self.pipeline = rs.pipeline()
        self.profile = None
        os.makedirs(self.output_dir, exist_ok=True)

    # -- setup section --
    def open(self) -> None:
        cfg = rs.config()
        cfg.enable_device_from_file(self.input_bag, repeat_playback=False)
        self.profile = self.pipeline.start(cfg)
        try:
            self.profile.get_device().as_playback().set_real_time(False)
        except Exception:
            pass
        if self.show_preview:
            cv2.namedWindow("bag_to_png", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("bag_to_png", self.win_w, self.win_h)

    # -- processing section --
    @staticmethod
    def frame_to_bgr(frame: rs.video_frame) -> np.ndarray:
        arr = np.asanyarray(frame.get_data())
        # Most common color format is RGB8; convert to BGR for OpenCV.
        if frame.get_profile().format() == rs.format.rgb8:
            arr = arr[:, :, ::-1].copy()
        return arr

    def process(self) -> int:
        count = 0
        while True:
            try:
                frames = self.pipeline.wait_for_frames()
            except RuntimeError:
                break

            color = frames.get_color_frame()
            if not color:
                continue

            img = self.frame_to_bgr(color)
            count += 1

            out_path = os.path.join(self.output_dir, f"frame_{count}.png")
            cv2.imwrite(out_path, img)

            if self.show_preview:
                disp = cv2.resize(img, (self.win_w, self.win_h), interpolation=cv2.INTER_AREA)
                cv2.putText(disp, f"Saved: cavity_{count}.png",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2, cv2.LINE_AA)
                cv2.imshow("bag_to_png", disp)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    break

            if self.max_frames is not None and count >= self.max_frames:
                break

        return count

    # -- teardown section --
    def close(self) -> None:
        try:
            self.pipeline.stop()
        finally:
            if self.show_preview:
                cv2.destroyAllWindows()

    def run(self) -> int:
        self.open()
        try:
            return self.process()
        finally:
            self.close()


def main() -> None:
    paths = Paths.from_repo_root()
    input_bag = os.path.join(paths.bags_dir, INPUT_BAG_NAME)
    output_dir = os.path.join(paths.root, "original", OUTPUT_SUBDIR)

    if not os.path.isfile(input_bag):
        raise FileNotFoundError(f"Missing .bag: {input_bag}")

    extractor = BagToPNG(
        input_bag=input_bag,
        output_dir=output_dir,
        window_size=WINDOW_SIZE,
        max_frames=MAX_FRAMES,
        show_preview=SHOW_PREVIEW,
    )
    saved = extractor.run()
    print(f"Saved {saved} frame(s) to {output_dir}")


if __name__ == "__main__":
    main()
