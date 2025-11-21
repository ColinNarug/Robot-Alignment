from __future__ import annotations
from pathlib import Path
from typing import Dict, Any, Iterable, Tuple
import yaml
import numpy as np
import torch
import segmentation_models_pytorch as smp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- Config / runtime ----------

def load_config(path: str | Path) -> Dict[str, Any]:
    """Load YAML config and fill defaults."""
    with open(path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    cfg.setdefault("paths", {})
    cfg["paths"].setdefault("data_root", "./dataset")
    cfg["paths"].setdefault("results_dir", "./results_256")
    cfg["paths"].setdefault("model_path", "./models/unet_256.pth")

    cfg.setdefault("dataset", {})
    cfg["dataset"].setdefault("resize", [256, 256])

    cfg.setdefault("model", {})
    cfg["model"].setdefault("encoder_name", "resnet34")
    cfg["model"].setdefault("encoder_weights", "imagenet")
    cfg["model"].setdefault("in_channels", 3)

    cfg.setdefault("inference", {})
    cfg["inference"].setdefault("pred_threshold", 0.7)
    cfg["inference"].setdefault("top_k", 10)

    cfg.setdefault("training", {})
    tr = cfg["training"]
    tr.setdefault("learning_rate", 2e-4)
    tr.setdefault("batch_size", 20)
    tr.setdefault("epochs", 30)
    tr.setdefault("val_frac", 0.2)
    tr.setdefault("seed", 1)
    tr.setdefault("device", "cuda")
    tr.setdefault("num_workers", 4)
    tr.setdefault("pin_memory", True)
    tr.setdefault("checkpoint_every", 10)
    tr.setdefault("analysis_every", 10)
    tr.setdefault("warm_start", False)
    return cfg


def ensure_dir(p: str | Path) -> Path:
    """Create folder if missing and return it."""
    path = Path(p)
    path.mkdir(parents=True, exist_ok=True)
    return path


def device_from(cfg: Dict[str, Any]) -> torch.device:
    """Return torch device from config (fallback to cpu if cuda not available)."""
    want = cfg["training"].get("device", "cuda")
    if want == "cuda" and not torch.cuda.is_available():
        return torch.device("cpu")
    return torch.device(want)


def set_seed(seed: int) -> None:
    """Set Python/NumPy/Torch seeds and enable cuDNN autotune."""
    import random
    random.seed(seed); np.random.seed(seed)
    torch.manual_seed(seed); torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = True


# ---------- Model ----------

def build_unet(cfg: Dict[str, Any], device: torch.device) -> torch.nn.Module:
    """Build SMP U-Net (logits)."""
    m = cfg["model"]
    net = smp.Unet(
        encoder_name=m["encoder_name"],
        encoder_weights=m["encoder_weights"],
        in_channels=int(m["in_channels"]),
        classes=1,
        activation=None,
    ).to(device)
    return net


def load_weights(model: torch.nn.Module, path: str | Path, device: torch.device, strict: bool = True) -> None:
    """Load state_dict from disk into model."""
    state = torch.load(Path(path), map_location=device)
    model.load_state_dict(state, strict=strict)


# ---------- Small numerics ----------

def k_closest_idx(values: np.ndarray, target: float, k: int) -> list[int]:
    """Indices of k entries closest to target."""
    if values.size == 0 or k <= 0:
        return []
    k = min(k, values.size)
    return np.argsort(np.abs(values - target), kind="mergesort")[:k].tolist()


# ---------- Plot helpers (small, focused) ----------

def save_histogram(data: Iterable[float], path: Path, title: str, xlabel: str,
                   vlines: Iterable[Tuple[float, str, str]] = ()) -> None:
    """Save a histogram with optional vertical lines (value, style, label)."""
    arr = np.asarray(list(data), dtype=float)
    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(1, 1, 1)
    ax.hist(arr, bins=30, alpha=0.85, edgecolor="black")
    for val, style, label in vlines:
        ax.axvline(val, linestyle=style, linewidth=2, label=label)
    ax.set_title(title); ax.set_xlabel(xlabel); ax.set_ylabel("Frequency")
    if vlines: ax.legend()
    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)


def save_grid(images: list[np.ndarray], row_titles: list[str], K: int, path: Path) -> None:
    """Save a 6xK grid given a flat list of 6*K images and row titles."""
    if K == 0: return
    fig = plt.figure(figsize=(4 * K, 18))
    for i, im in enumerate(images, 1):
        ax = fig.add_subplot(6, K, i)
        ax.imshow(im if im.ndim == 3 else im, cmap=None if im.ndim == 3 else "gray")
        ax.axis("off")
        if (i - 1) % K == 0:
            ax.set_title(row_titles[(i - 1) // K])
    fig.tight_layout(); fig.savefig(path, dpi=150); plt.close(fig)
