# Calibration Quality Report

- Session folder: `intrinsics_20260312_155258_327122073214_1920x1080`
- Camera serial: `327122073214`
- Image size: `1920 x 1080`
- Board inner corners: `24 x 17`
- Square size: `0.0075 m`
- Captured samples used: `22`
- OpenCV RMS: `0.537878 px`
- Mean reprojection error: `0.464759 px`
- Undistort alpha: `1`
- Crop undistorted image: `true`

## Intrinsics

```text
K =
 1343.69947274     0.00000000   983.75820745
    0.00000000  1342.51682957   557.47669371
    0.00000000     0.00000000     1.00000000

D =
    0.17776410    -0.57063101     0.00109004     0.00018672     0.51938888
```

## Intrinsic Parameter Standard Deviations

```text
    0.23533660
    0.27243493
    0.33350039
    0.26364219
    0.00103754
    0.00445445
    0.00007397
    0.00008962
    0.00562974
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

- OpenCV per-view reprojection error mean: `0.529333 px`
- OpenCV per-view reprojection error median: `0.527358 px`
- OpenCV per-view reprojection error max: `0.737013 px`

## Per-Sample Checks

| Sample | Reproj mean (px) | Reproj max (px) | Raw border RMS (px) | Corrected border RMS (px) | Border improvement (px) | Improvement (%) | Raw lines | Corners | Corrected |
|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| 1 | 0.359341 | 1.0524 | 0.478197 | 0.190497 | 0.2877 | 60.1635 | `sample_0001.png` | `sample_0001.png` | `sample_0001.png` |
| 2 | 0.499696 | 2.90654 | 0.604302 | 0.323564 | 0.280738 | 46.4566 | `sample_0002.png` | `sample_0002.png` | `sample_0002.png` |
| 3 | 0.489476 | 1.53496 | 0.312412 | 0.282568 | 0.0298443 | 9.55288 | `sample_0003.png` | `sample_0003.png` | `sample_0003.png` |
| 4 | 0.51311 | 1.41144 | 0.703389 | 0.323364 | 0.380024 | 54.0277 | `sample_0004.png` | `sample_0004.png` | `sample_0004.png` |
| 5 | 0.477381 | 1.3664 | 0.822025 | 0.314607 | 0.507418 | 61.7278 | `sample_0005.png` | `sample_0005.png` | `sample_0005.png` |
| 6 | 0.403857 | 1.15399 | 0.675093 | 0.240223 | 0.434871 | 64.4164 | `sample_0006.png` | `sample_0006.png` | `sample_0006.png` |
| 7 | 0.463955 | 1.13391 | 0.493702 | 0.23526 | 0.258442 | 52.3477 | `sample_0007.png` | `sample_0007.png` | `sample_0007.png` |
| 8 | 0.512434 | 1.41265 | 0.590763 | 0.272814 | 0.317949 | 53.82 | `sample_0008.png` | `sample_0008.png` | `sample_0008.png` |
| 9 | 0.443321 | 1.20495 | 0.706616 | 0.325048 | 0.381568 | 53.9994 | `sample_0009.png` | `sample_0009.png` | `sample_0009.png` |
| 10 | 0.384669 | 1.2155 | 0.557983 | 0.22672 | 0.331263 | 59.3679 | `sample_0010.png` | `sample_0010.png` | `sample_0010.png` |
| 11 | 0.412254 | 1.43453 | 0.665755 | 0.260463 | 0.405292 | 60.8771 | `sample_0011.png` | `sample_0011.png` | `sample_0011.png` |
| 12 | 0.419967 | 1.41589 | 0.494596 | 0.257852 | 0.236744 | 47.8661 | `sample_0012.png` | `sample_0012.png` | `sample_0012.png` |
| 13 | 0.512489 | 1.54904 | 0.403054 | 0.35088 | 0.0521737 | 12.9446 | `sample_0013.png` | `sample_0013.png` | `sample_0013.png` |
| 14 | 0.527826 | 1.42176 | 0.363577 | 0.246154 | 0.117424 | 32.2967 | `sample_0014.png` | `sample_0014.png` | `sample_0014.png` |
| 15 | 0.39345 | 1.16468 | 0.542869 | 0.220916 | 0.321953 | 59.3058 | `sample_0015.png` | `sample_0015.png` | `sample_0015.png` |
| 16 | 0.603676 | 2.06994 | 0.572573 | 0.469576 | 0.102997 | 17.9885 | `sample_0016.png` | `sample_0016.png` | `sample_0016.png` |
| 17 | 0.556045 | 1.85448 | 0.671561 | 0.517833 | 0.153728 | 22.8911 | `sample_0017.png` | `sample_0017.png` | `sample_0017.png` |
| 18 | 0.424208 | 1.40851 | 0.305531 | 0.16617 | 0.139361 | 45.6128 | `sample_0018.png` | `sample_0018.png` | `sample_0018.png` |
| 19 | 0.3577 | 0.813272 | 0.207076 | 0.128107 | 0.0789693 | 38.1354 | `sample_0019.png` | `sample_0019.png` | `sample_0019.png` |
| 20 | 0.305804 | 0.946453 | 0.32386 | 0.130925 | 0.192935 | 59.5737 | `sample_0020.png` | `sample_0020.png` | `sample_0020.png` |
| 21 | 0.646507 | 1.78248 | 0.678289 | 0.491093 | 0.187196 | 27.5983 | `sample_0021.png` | `sample_0021.png` | `sample_0021.png` |
| 22 | 0.517534 | 1.59721 | 0.612271 | 0.431713 | 0.180558 | 29.4899 | `sample_0022.png` | `sample_0022.png` | `sample_0022.png` |

## Aggregate Straightness Check

- Mean raw border RMS: `0.535704 px`
- Mean corrected border RMS: `0.291198 px`
- Mean border straightness improvement: `0.244507 px`
- Mean border straightness improvement: `44.1118 %`

## Qualitative Interpretation

The validation bundle is intended to answer two questions:

1. **Do the fitted parameters reproduce the observed corner locations well?**  
   This is summarized by the OpenCV RMS value, the mean reprojection error, and the per-sample reprojection statistics.

2. **Do straight physical chessboard borders become straighter after undistortion?**  
   This is summarized by the raw-versus-corrected border-line RMS residuals and the saved validation images.

Observed result: the corrected images reduce the mean border-line residual by `0.244507 px` on average (`44.1118 %` improvement).

Use the images in `01_uncorrected_with_lines`, `02_corner_detection`, and `03_corrected_image` together with the reprojection numbers above to judge calibration quality for this run.
