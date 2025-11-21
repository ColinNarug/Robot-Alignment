#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import argparse
import time
from pathlib import Path
from typing import Tuple, List

import torch
from torch import nn, optim
from torch.utils.data import DataLoader, random_split
from tqdm import tqdm

# Local utilities (package lives in ./training_utilities)
from training_utilities.holes_dataset import HolesDataset
from training_utilities.utils import load_config, ensure_dir, set_seed, device_from, build_unet
import training_utilities.train_analysis as train_analysis  
from training_utilities.model_analysis import ModelAnalyzer

"""
Train an SMP U-Net
Overview
--------
- Dataset :        ./train_dataset/{roi,masks}
- Results : ./runs/result_<k>/  (k = first free integer)
    ├── models/     (checkpoints + final weights)
    └── analysis/   (loss_curve.png, val_scatter.png, overfit_gap.png)
- Config (fixed path):    ./train_config.yaml 
- Optional warm start:    paths.finetune_from (empty = scratch)
"""

# ----------------------------
# Run directory helpers
# ----------------------------
def _next_run_dir(base: Path) -> Path:
    """
    Return ./runs/result_k where k is the first non-existing positive integer.
    Directory is NOT created here.
    """
    runs_dir = base
    runs_dir.mkdir(parents=True, exist_ok=True)  # ensure ./runs exists
    k = 1
    while True:
        d = runs_dir / f"result_{k}"
        if not d.exists():
            return d
        k += 1


class Trainer:
    """U-Net trainer with BCEWithLogits loss and per-epoch analysis."""

    def __init__(self, cfg: dict) -> None:
        """Initialize device, run dirs, dataset/dataloaders, model/opt/loss, and logs."""
        self.cfg = cfg
        self.device = device_from(cfg)
        set_seed(int(cfg["training"]["seed"]))

        # ----- fixed dataset root -----
        self.data_root = Path("./train_dataset").resolve()

        # ----- auto-increment run directory: ./runs/result_k -----
        self.run_dir = _next_run_dir(Path("./runs").resolve())
        self.models_dir = ensure_dir(self.run_dir / "models")
        self.analysis_dir = ensure_dir(self.run_dir / "analysis")

        # ----- dataset and split -----
        ds = HolesDataset(
            root_path=str(self.data_root),                   # expects roi/ and masks/
            in_channels=int(cfg["model"]["in_channels"]),
            resize= [256, 256]
        )
        n_val = int(len(ds) * float(cfg["training"]["val_over_total_frac"]))
        n_train = len(ds) - n_val
        g = torch.Generator().manual_seed(int(cfg["training"]["seed"]))
        self.train_ds, self.val_ds = random_split(ds, [n_train, n_val], generator=g)

        # ----- dataloaders -----
        bs = int(cfg["training"]["batch_size"])
        nw = int(cfg["training"]["num_workers"])
        pin = bool(cfg["training"]["pin_memory"]) and self.device.type == "cuda"
        self.train_dl = DataLoader(self.train_ds, batch_size=bs, shuffle=True,  num_workers=nw, pin_memory=pin)
        self.val_dl   = DataLoader(self.val_ds,   batch_size=bs, shuffle=False, num_workers=nw, pin_memory=pin)

        # ----- model -----
        self.model = build_unet(cfg, self.device)

        # ----- optional warm start (fine-tuning) -----
        finetune_path = (cfg["paths"].get("finetune_from") or "").strip()
        if bool(cfg["training"]["warm_start"]) and finetune_path:
            p = Path(finetune_path)
            if p.exists():
                state = torch.load(p, map_location=self.device)
                self.model.load_state_dict(state, strict=False)
                print(f"Warm-started from: {p}")
            else:
                print(f"[WARNING] finetune_from not found: {p} (training from scratch)")

        # ----- optimizer and loss -----
        self.opt  = optim.AdamW(self.model.parameters(), lr=float(cfg["training"]["learning_rate"]))
        self.crit = nn.BCEWithLogitsLoss()

        # ----- logs for analysis -----
        self.train_losses: List[float] = []
        self.val_losses:   List[float] = []

        # ----- analysis smoothing -----
        self.smooth_window  = 5

        # ----- checkpoint cadence -----
        self.ckpt_every = int(cfg["training"]["checkpoint_every"])

        print(f"\n------ Starting the training ------")

    # ----------------------------
    # steps / epochs
    # ----------------------------
    def _step(self, batch) -> torch.Tensor:
        """Compute BCEWithLogits loss for one batch."""
        x, y = batch
        x = x.to(self.device).float()
        y = y.to(self.device).float()
        logits = self.model(x)
        return self.crit(logits, y)

    def _run_epoch(self, is_train: bool) -> float:
        """Run one epoch (train or val) and return mean loss."""
        loader = self.train_dl if is_train else self.val_dl
        self.model.train(mode=is_train)
        tot, n = 0.0, 0
        for batch in tqdm(loader, leave=False, desc="Train" if is_train else " Val "):
            with torch.set_grad_enabled(is_train):
                loss = self._step(batch)
                if is_train:
                    self.opt.zero_grad(set_to_none=True)
                    loss.backward()
                    self.opt.step()
            tot += float(loss.item())
            n += 1
        return tot / max(1, n)

    def _maybe_checkpoint(self, epoch_idx: int) -> None:
        """Save a checkpoint every ckpt_every epochs."""
        if self.ckpt_every and (epoch_idx + 1) % self.ckpt_every == 0:
            ckpt = self.models_dir / f"unet_epoch_{epoch_idx+1:03d}.pth"
            torch.save(self.model.state_dict(), ckpt)
            print(f"---> Saved checkpoint: {ckpt.name}")

    def _run_analysis(self) -> None:
        """Always generate plots into the analysis folder (no CSV)."""
        try:
            train_analysis.main(
                train_losses=self.train_losses,
                val_losses=self.val_losses,
                out_dir=self.analysis_dir,
                smooth_window=self.smooth_window,
            )
        except Exception as e:
            print(f"[WARN] Analysis failed: {e}")

    # ----------------------------
    # training loop
    # ----------------------------
    def run(self) -> None:
        """Full training loop with per-epoch logging, checkpoints, and ALWAYS-on analysis."""
        E = int(self.cfg["training"]["epochs"])
        t0_all = time.time()
        for e in range(E):
            t0 = time.time()
            tr = self._run_epoch(is_train=True)
            vl = self._run_epoch(is_train=False)
            dt = time.time() - t0

            self.train_losses.append(tr)
            self.val_losses.append(vl)

            print(f"epoch {e+1:02d}/{E:02d} | train loss = {tr:.4f} | validation loss = {vl:.4f} | epoch training time {dt:.1f} [s]")
            self._maybe_checkpoint(e)

            # ALWAYS run analysis after each epoch
            self._run_analysis()

        # Save final weights
        final = self.models_dir / "unet_final.pth"
        torch.save(self.model.state_dict(), final)
        print(f"------ Training finished in {(time.time()-t0_all)/60:.1f} [min] ------")
        print(f"Running analysis on the best model...")


        # Run analysis once more at the end
        self._run_analysis()

        # Create a subfolder inside the run's analysis directory to keep things tidy
        model_eval_dir = ensure_dir(self.analysis_dir / "model_eval")

        # Read thresholds from config if present, else fallback
        thr = float(self.cfg.get("inference", {}).get("pred_threshold", 0.7))

        # Run model analysis on the validation split
        analyzer = ModelAnalyzer(
            model=self.model,
            dataset=self.val_ds,           # or self.train_ds if you want train-set analysis
            device=self.device,
            out_dir=model_eval_dir,
            threshold=thr,
            top_k=5,
        )
        summary = analyzer.run()
        print(f"... done!")


def main() -> None:
    """Parse --config (defaults to ./train_config.yaml), load cfg, and run training."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", type=str, default="train_config.yaml")
    args = ap.parse_args()

    cfg = load_config(args.config)
    Trainer(cfg).run()


if __name__ == "__main__":
    main()
