# Calibration Quality Report

- Session folder: `intrinsics_20260312_160742_327122073214_1920x1080`
- Camera serial: `327122073214`
- Image size: `1920 x 1080`
- Board inner corners: `24 x 17`
- Square size: `0.0075 m`
- Captured samples used: `23`
- OpenCV RMS: `0.515274 px`
- Mean reprojection error: `0.444061 px`
- Undistort alpha: `1`
- Crop undistorted image: `true`

## Intrinsics

```text
K =
 1337.48357382     0.00000000   953.46206759
    0.00000000  1335.62431109   547.27329079
    0.00000000     0.00000000     1.00000000

D =
    0.14922619    -0.47518364     0.00221655     0.00042870     0.45092808
```

## Intrinsic Parameter Standard Deviations

```text
    0.22649598
    0.26238238
    0.35546598
    0.26741134
    0.00107614
    0.00497419
    0.00007600
    0.00009720
    0.00676071
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

- OpenCV per-view reprojection error mean: `0.505666 px`
- OpenCV per-view reprojection error median: `0.493619 px`
- OpenCV per-view reprojection error max: `0.721373 px`

## Per-Sample Checks

| Sample | Reproj mean (px) | Reproj max (px) | Raw border RMS (px) | Corrected border RMS (px) | Border improvement (px) | Improvement (%) | Raw lines | Corners | Corrected |
|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| 1 | 0.313086 | 0.79157 | 0.421135 | 0.171318 | 0.249816 | 59.3198 | `sample_0001.png` | `sample_0001.png` | `sample_0001.png` |
| 2 | 0.457913 | 1.42983 | 0.488289 | 0.223319 | 0.26497 | 54.2651 | `sample_0002.png` | `sample_0002.png` | `sample_0002.png` |
| 3 | 0.401382 | 1.37086 | 0.218981 | 0.244257 | -0.0252762 | -11.5426 | `sample_0003.png` | `sample_0003.png` | `sample_0003.png` |
| 4 | 0.483969 | 1.36722 | 0.568843 | 0.259312 | 0.309531 | 54.4141 | `sample_0004.png` | `sample_0004.png` | `sample_0004.png` |
| 5 | 0.432437 | 1.69143 | 0.62313 | 0.207275 | 0.415854 | 66.7364 | `sample_0005.png` | `sample_0005.png` | `sample_0005.png` |
| 6 | 0.421969 | 1.28279 | 0.562484 | 0.198588 | 0.363896 | 64.6945 | `sample_0006.png` | `sample_0006.png` | `sample_0006.png` |
| 7 | 0.463266 | 1.23816 | 0.456585 | 0.243274 | 0.213311 | 46.7189 | `sample_0007.png` | `sample_0007.png` | `sample_0007.png` |
| 8 | 0.518169 | 1.31788 | 0.549179 | 0.273937 | 0.275242 | 50.1188 | `sample_0008.png` | `sample_0008.png` | `sample_0008.png` |
| 9 | 0.473992 | 1.20937 | 0.558726 | 0.271658 | 0.287068 | 51.379 | `sample_0009.png` | `sample_0009.png` | `sample_0009.png` |
| 10 | 0.343844 | 1.05633 | 0.495158 | 0.189407 | 0.30575 | 61.7481 | `sample_0010.png` | `sample_0010.png` | `sample_0010.png` |
| 11 | 0.396293 | 1.58812 | 0.502154 | 0.24878 | 0.253374 | 50.4574 | `sample_0011.png` | `sample_0011.png` | `sample_0011.png` |
| 12 | 0.41354 | 1.31068 | 0.376036 | 0.290617 | 0.0854193 | 22.7157 | `sample_0012.png` | `sample_0012.png` | `sample_0012.png` |
| 13 | 0.541893 | 2.61113 | 0.326404 | 0.373036 | -0.0466321 | -14.2866 | `sample_0013.png` | `sample_0013.png` | `sample_0013.png` |
| 14 | 0.512663 | 1.49118 | 0.278066 | 0.25801 | 0.0200555 | 7.21251 | `sample_0014.png` | `sample_0014.png` | `sample_0014.png` |
| 15 | 0.407637 | 1.87771 | 0.421308 | 0.236142 | 0.185166 | 43.9503 | `sample_0015.png` | `sample_0015.png` | `sample_0015.png` |
| 16 | 0.414239 | 1.6722 | 0.412885 | 0.233455 | 0.17943 | 43.4576 | `sample_0016.png` | `sample_0016.png` | `sample_0016.png` |
| 17 | 0.532368 | 2.21792 | 0.410782 | 0.453681 | -0.0428989 | -10.4432 | `sample_0017.png` | `sample_0017.png` | `sample_0017.png` |
| 18 | 0.509827 | 1.79989 | 0.486322 | 0.509327 | -0.0230051 | -4.73044 | `sample_0018.png` | `sample_0018.png` | `sample_0018.png` |
| 19 | 0.429396 | 1.46757 | 0.220436 | 0.16181 | 0.0586263 | 26.5956 | `sample_0019.png` | `sample_0019.png` | `sample_0019.png` |
| 20 | 0.303722 | 0.779509 | 0.195035 | 0.122669 | 0.0723661 | 37.1042 | `sample_0020.png` | `sample_0020.png` | `sample_0020.png` |
| 21 | 0.272555 | 0.754387 | 0.218038 | 0.110522 | 0.107516 | 49.3107 | `sample_0021.png` | `sample_0021.png` | `sample_0021.png` |
| 22 | 0.643386 | 1.68566 | 0.553791 | 0.399812 | 0.153979 | 27.8045 | `sample_0022.png` | `sample_0022.png` | `sample_0022.png` |
| 23 | 0.525857 | 1.47769 | 0.514272 | 0.356562 | 0.15771 | 30.6666 | `sample_0023.png` | `sample_0023.png` | `sample_0023.png` |

## Aggregate Straightness Check

- Mean raw border RMS: `0.42861 px`
- Mean corrected border RMS: `0.262468 px`
- Mean border straightness improvement: `0.166142 px`
- Mean border straightness improvement: `35.1159 %`

## Qualitative Interpretation

The validation bundle is intended to answer two questions:

1. **Do the fitted parameters reproduce the observed corner locations well?**  
   This is summarized by the OpenCV RMS value, the mean reprojection error, and the per-sample reprojection statistics.

2. **Do straight physical chessboard borders become straighter after undistortion?**  
   This is summarized by the raw-versus-corrected border-line RMS residuals and the saved validation images.

Observed result: the corrected images reduce the mean border-line residual by `0.166142 px` on average (`35.1159 %` improvement).

Use the images in `01_uncorrected_with_lines`, `02_corner_detection`, and `03_corrected_image` together with the reprojection numbers above to judge calibration quality for this run.
