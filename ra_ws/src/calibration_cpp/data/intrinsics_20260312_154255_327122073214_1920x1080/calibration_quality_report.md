# Calibration Quality Report

- Session folder: `intrinsics_20260312_154255_327122073214_1920x1080`
- Camera serial: `327122073214`
- Image size: `1920 x 1080`
- Board inner corners: `24 x 17`
- Square size: `0.0075 m`
- Captured samples used: `22`
- OpenCV RMS: `0.52934 px`
- Mean reprojection error: `0.452408 px`
- Undistort alpha: `1`
- Crop undistorted image: `true`

## Intrinsics

```text
K =
 1338.01375682     0.00000000   951.38744820
    0.00000000  1336.08253716   547.81304152
    0.00000000     0.00000000     1.00000000

D =
    0.15056044    -0.48206676     0.00235820    -0.00008007     0.45802746
```

## Intrinsic Parameter Standard Deviations

```text
    0.23169883
    0.27263969
    0.35542449
    0.26263652
    0.00104958
    0.00458914
    0.00007380
    0.00009715
    0.00590950
    0.00000000
    0.00000000
    0.00000000
    0.00000000
    0.00000000
    0.00000000
    0.00000000
    0.00000000
    0.00000000
```

## Aggregate Quantitative Checks

- OpenCV per-view reprojection error mean: `0.518642 px`
- OpenCV per-view reprojection error median: `0.503736 px`
- OpenCV per-view reprojection error max: `0.724952 px`

## Per-Sample Checks

| Sample | Reproj mean (px) | Reproj max (px) | Raw border RMS (px) | Corrected border RMS (px) | Border improvement (px) | Improvement (%) | Raw lines | Corners | Corrected |
|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| 1 | 0.336837 | 0.973694 | 0.341011 | 0.220417 | 0.120594 | 35.3637 | `sample_0001.png` | `sample_0001.png` | `sample_0001.png` |
| 2 | 0.461362 | 2.87632 | 0.481491 | 0.290864 | 0.190627 | 39.591 | `sample_0002.png` | `sample_0002.png` | `sample_0002.png` |
| 3 | 0.417223 | 1.3449 | 0.228493 | 0.249777 | -0.021284 | -9.31499 | `sample_0003.png` | `sample_0003.png` | `sample_0003.png` |
| 4 | 0.487416 | 1.38434 | 0.568251 | 0.261474 | 0.306776 | 53.9861 | `sample_0004.png` | `sample_0004.png` | `sample_0004.png` |
| 5 | 0.435932 | 1.22833 | 0.641745 | 0.210337 | 0.431409 | 67.2243 | `sample_0005.png` | `sample_0005.png` | `sample_0005.png` |
| 6 | 0.437429 | 1.16794 | 0.672616 | 0.231434 | 0.441182 | 65.592 | `sample_0006.png` | `sample_0006.png` | `sample_0006.png` |
| 7 | 0.44775 | 1.09181 | 0.460172 | 0.231634 | 0.228539 | 49.6637 | `sample_0007.png` | `sample_0007.png` | `sample_0007.png` |
| 8 | 0.506337 | 1.3991 | 0.569397 | 0.281536 | 0.287861 | 50.5555 | `sample_0008.png` | `sample_0008.png` | `sample_0008.png` |
| 9 | 0.492118 | 1.14949 | 0.554341 | 0.307234 | 0.247107 | 44.5767 | `sample_0009.png` | `sample_0009.png` | `sample_0009.png` |
| 10 | 0.349052 | 1.10102 | 0.469593 | 0.215207 | 0.254385 | 54.1715 | `sample_0010.png` | `sample_0010.png` | `sample_0010.png` |
| 11 | 0.411411 | 1.70443 | 0.521191 | 0.255093 | 0.266098 | 51.0557 | `sample_0011.png` | `sample_0011.png` | `sample_0011.png` |
| 12 | 0.437904 | 1.95068 | 0.407423 | 0.317172 | 0.0902516 | 22.1518 | `sample_0012.png` | `sample_0012.png` | `sample_0012.png` |
| 13 | 0.585931 | 2.47276 | 0.331244 | 0.391508 | -0.060264 | -18.1932 | `sample_0013.png` | `sample_0013.png` | `sample_0013.png` |
| 14 | 0.425943 | 1.41696 | 0.42517 | 0.24121 | 0.18396 | 43.2673 | `sample_0014.png` | `sample_0014.png` | `sample_0014.png` |
| 15 | 0.422546 | 2.13812 | 0.417493 | 0.253875 | 0.163618 | 39.1906 | `sample_0015.png` | `sample_0015.png` | `sample_0015.png` |
| 16 | 0.54831 | 2.67807 | 0.480721 | 0.465419 | 0.015302 | 3.18313 | `sample_0016.png` | `sample_0016.png` | `sample_0016.png` |
| 17 | 0.560654 | 1.74923 | 0.525591 | 0.548016 | -0.022425 | -4.26663 | `sample_0017.png` | `sample_0017.png` | `sample_0017.png` |
| 18 | 0.445506 | 1.54461 | 0.231238 | 0.164279 | 0.066959 | 28.9567 | `sample_0018.png` | `sample_0018.png` | `sample_0018.png` |
| 19 | 0.299682 | 0.736509 | 0.163378 | 0.107728 | 0.0556497 | 34.0619 | `sample_0019.png` | `sample_0019.png` | `sample_0019.png` |
| 20 | 0.273786 | 0.760459 | 0.201174 | 0.109479 | 0.091695 | 45.58 | `sample_0020.png` | `sample_0020.png` | `sample_0020.png` |
| 21 | 0.647929 | 1.6899 | 0.596291 | 0.417319 | 0.178972 | 30.0143 | `sample_0021.png` | `sample_0021.png` | `sample_0021.png` |
| 22 | 0.521907 | 1.56128 | 0.557334 | 0.410967 | 0.146367 | 26.2619 | `sample_0022.png` | `sample_0022.png` | `sample_0022.png` |

## Aggregate Straightness Check

- Mean raw border RMS: `0.447516 px`
- Mean corrected border RMS: `0.280999 px`
- Mean border straightness improvement: `0.166517 px`
- Mean border straightness improvement: `34.2124 %`

## Qualitative Interpretation

The validation bundle is intended to answer two questions:

1. **Do the fitted parameters reproduce the observed corner locations well?**  
   This is summarized by the OpenCV RMS value, the mean reprojection error, and the per-sample reprojection statistics.

2. **Do straight physical chessboard borders become straighter after undistortion?**  
   This is summarized by the raw-versus-corrected border-line RMS residuals and the saved validation images.

Observed result: the corrected images reduce the mean border-line residual by `0.166517 px` on average (`34.2124 %` improvement).

Use the images in `01_uncorrected_with_lines`, `02_corner_detection`, and `03_corrected_image` together with the reprojection numbers above to judge calibration quality for this run.
