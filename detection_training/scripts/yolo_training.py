#!/usr/bin/env python3
"""Train a YOLO model with a dataset config (minimal version)."""

from ultralytics import YOLO
from paths import Paths
import os


def main() -> None:
    paths = Paths.from_repo_root("configTraining.yaml", "yolo11n.pt")

    if not os.path.isfile(paths.weights_path):
        raise FileNotFoundError(f"Missing weights: {paths.weights_path}")
    if not os.path.isfile(paths.config_path):
        raise FileNotFoundError(f"Missing data config: {paths.config_path}")

    # Train the model
    # Note: you can add more parameters here, e.g., epochs, imgsz...
    YOLO(paths.weights_path).train(
        data=paths.config_path,
        epochs=150,
        imgsz=960,
    )


if __name__ == "__main__":
    main()
