#!/usr/bin/env python3
"""Project paths resolved from the repository root."""

from dataclasses import dataclass
import os


@dataclass(frozen=True)
class Paths:
    root: str
    config_path: str
    weights_path: str
    images_train: str
    labels_train: str
    bags_dir: str
    runs_dir: str

    @staticmethod
    def from_repo_root(
        data_config_name: str = "configTraining.yaml",
        weights_name: str = "yolo11n.pt",
    ) -> "Paths":
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return Paths(
            root=root,
            config_path=os.path.join(root, "config", data_config_name),
            weights_path=os.path.join(root, weights_name),
            images_train=os.path.join(root, "images", "train"),
            labels_train=os.path.join(root, "labels", "train"),
            bags_dir=os.path.join(root, "bags"),
            runs_dir=os.path.join(root, "runs"),
        )
