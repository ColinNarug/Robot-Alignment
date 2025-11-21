#!/usr/bin/env python3
"""Check that every image has a corresponding YOLO label and vice versa."""

import os
from paths import Paths


# ============================== #
# ======= TUNABLE PARAMS ======= #
# ============================== #
IMAGE_SUBDIR = "images/train"
LABEL_SUBDIR = "labels/train"
# ============================== #


class DatasetChecker:
    def __init__(self, image_dir: str, label_dir: str) -> None:
        self.image_dir = image_dir
        self.label_dir = label_dir

    def check_mismatched_files(self) -> tuple[list[str], list[str]]:
        image_files = {
            os.path.splitext(f)[0]
            for f in os.listdir(self.image_dir)
            if f.lower().endswith(('.jpg', '.png', '.jpeg'))
        }
        label_files = {
            os.path.splitext(f)[0]
            for f in os.listdir(self.label_dir)
            if f.lower().endswith('.txt')
        }
        return (
            [img for img in image_files if img not in label_files],
            [lbl for lbl in label_files if lbl not in image_files],
        )

    def run(self) -> None:
        if not (os.path.exists(self.image_dir) and os.path.exists(self.label_dir)):
            print("One or both folders do not exist.")
            return

        missing_imgs, missing_labels = self.check_mismatched_files()

        if missing_imgs:
            print("Images without labels:")
            for f in missing_imgs:
                print(f)

        if missing_labels:
            print("\nLabels without images:")
            for f in missing_labels:
                print(f)

        if not missing_imgs and not missing_labels:
            print("All images and labels match ✅")


def main() -> None:
    paths = Paths.from_repo_root()
    image_dir = os.path.join(paths.root, IMAGE_SUBDIR)
    label_dir = os.path.join(paths.root, LABEL_SUBDIR)
    DatasetChecker(image_dir, label_dir).run()


if __name__ == "__main__":
    main()
