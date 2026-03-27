# Calibration Quality Report

- Session folder: `intrinsics_20260312_160059_050422071950_1920x1080`
- Camera serial: `050422071950`
- Image size: `1920 x 1080`
- Board inner corners: `24 x 17`
- Square size: `0.0075 m`
- Captured samples used: `22`
- OpenCV RMS: `0.533159 px`
- Mean reprojection error: `0.462541 px`
- Undistort alpha: `1`
- Crop undistorted image: `true`

## Intrinsics

```text
K =
 1344.13798978     0.00000000   983.91179795
    0.00000000  1342.91502914   557.61009740
    0.00000000     0.00000000     1.00000000

D =
    0.17646977    -0.56195860     0.00115304     0.00022068     0.50725683
```

## Intrinsic Parameter Standard Deviations

```text
    0.23633071
    0.27416321
    0.33622080
    0.26856935
    0.00103149
    0.00447346
    0.00007565
    0.00009060
    0.00570176
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

- OpenCV per-view reprojection error mean: `0.525392 px`
- OpenCV per-view reprojection error median: `0.53069 px`
- OpenCV per-view reprojection error max: `0.712033 px`

## Per-Sample Checks

| Sample | Reproj mean (px) | Reproj max (px) | Raw border RMS (px) | Corrected border RMS (px) | Border improvement (px) | Improvement (%) | Raw lines | Corners | Corrected |
|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| 1 | 0.363903 | 1.01825 | 0.482894 | 0.190499 | 0.292395 | 60.5505 | `sample_0001.png` | `sample_0001.png` | `sample_0001.png` |
| 2 | 0.514917 | 2.87944 | 0.614984 | 0.335559 | 0.279425 | 45.4361 | `sample_0002.png` | `sample_0002.png` | `sample_0002.png` |
| 3 | 0.497019 | 1.45758 | 0.32887 | 0.28149 | 0.0473798 | 14.4069 | `sample_0003.png` | `sample_0003.png` | `sample_0003.png` |
| 4 | 0.513674 | 1.41244 | 0.674562 | 0.280466 | 0.394097 | 58.4226 | `sample_0004.png` | `sample_0004.png` | `sample_0004.png` |
| 5 | 0.480734 | 1.48435 | 0.861598 | 0.296793 | 0.564805 | 65.5532 | `sample_0005.png` | `sample_0005.png` | `sample_0005.png` |
| 6 | 0.399021 | 1.23008 | 0.708407 | 0.25669 | 0.451717 | 63.7652 | `sample_0006.png` | `sample_0006.png` | `sample_0006.png` |
| 7 | 0.467786 | 1.10579 | 0.502367 | 0.23842 | 0.263948 | 52.5408 | `sample_0007.png` | `sample_0007.png` | `sample_0007.png` |
| 8 | 0.512442 | 1.33915 | 0.616254 | 0.283893 | 0.332361 | 53.9325 | `sample_0008.png` | `sample_0008.png` | `sample_0008.png` |
| 9 | 0.439993 | 1.24044 | 0.710317 | 0.308234 | 0.402083 | 56.6062 | `sample_0009.png` | `sample_0009.png` | `sample_0009.png` |
| 10 | 0.385291 | 1.25302 | 0.547535 | 0.224311 | 0.323224 | 59.0325 | `sample_0010.png` | `sample_0010.png` | `sample_0010.png` |
| 11 | 0.415512 | 1.41861 | 0.637313 | 0.247018 | 0.390296 | 61.2408 | `sample_0011.png` | `sample_0011.png` | `sample_0011.png` |
| 12 | 0.425545 | 1.39494 | 0.479267 | 0.278035 | 0.201232 | 41.9875 | `sample_0012.png` | `sample_0012.png` | `sample_0012.png` |
| 13 | 0.502529 | 1.57369 | 0.361183 | 0.345738 | 0.0154453 | 4.2763 | `sample_0013.png` | `sample_0013.png` | `sample_0013.png` |
| 14 | 0.517963 | 1.29875 | 0.363221 | 0.235904 | 0.127317 | 35.0521 | `sample_0014.png` | `sample_0014.png` | `sample_0014.png` |
| 15 | 0.386206 | 1.08843 | 0.528059 | 0.229329 | 0.29873 | 56.5714 | `sample_0015.png` | `sample_0015.png` | `sample_0015.png` |
| 16 | 0.622256 | 2.03912 | 0.516982 | 0.475598 | 0.041384 | 8.00492 | `sample_0016.png` | `sample_0016.png` | `sample_0016.png` |
| 17 | 0.535403 | 1.66839 | 0.625634 | 0.456949 | 0.168685 | 26.9623 | `sample_0017.png` | `sample_0017.png` | `sample_0017.png` |
| 18 | 0.418705 | 1.37957 | 0.315372 | 0.162872 | 0.1525 | 48.3556 | `sample_0018.png` | `sample_0018.png` | `sample_0018.png` |
| 19 | 0.361787 | 0.887392 | 0.252014 | 0.131448 | 0.120566 | 47.8409 | `sample_0019.png` | `sample_0019.png` | `sample_0019.png` |
| 20 | 0.306685 | 0.919857 | 0.320261 | 0.113612 | 0.206648 | 64.525 | `sample_0020.png` | `sample_0020.png` | `sample_0020.png` |
| 21 | 0.614412 | 1.69507 | 0.584904 | 0.458675 | 0.126229 | 21.5811 | `sample_0021.png` | `sample_0021.png` | `sample_0021.png` |
| 22 | 0.494112 | 1.50071 | 0.571852 | 0.365934 | 0.205918 | 36.0089 | `sample_0022.png` | `sample_0022.png` | `sample_0022.png` |

## Aggregate Straightness Check

- Mean raw border RMS: `0.527448 px`
- Mean corrected border RMS: `0.281703 px`
- Mean border straightness improvement: `0.245745 px`
- Mean border straightness improvement: `44.6661 %`

## Qualitative Interpretation

The validation bundle is intended to answer two questions:

1. **Do the fitted parameters reproduce the observed corner locations well?**  
   This is summarized by the OpenCV RMS value, the mean reprojection error, and the per-sample reprojection statistics.

2. **Do straight physical chessboard borders become straighter after undistortion?**  
   This is summarized by the raw-versus-corrected border-line RMS residuals and the saved validation images.

Observed result: the corrected images reduce the mean border-line residual by `0.245745 px` on average (`44.6661 %` improvement).

Use the images in `01_uncorrected_with_lines`, `02_corner_detection`, and `03_corrected_image` together with the reprojection numbers above to judge calibration quality for this run.
