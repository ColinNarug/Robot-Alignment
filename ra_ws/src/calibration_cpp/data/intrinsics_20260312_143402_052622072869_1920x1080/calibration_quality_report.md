# Calibration Quality Report

- Session folder: `intrinsics_20260312_143402_052622072869_1920x1080`
- Camera serial: `052622072869`
- Image size: `1920 x 1080`
- Board inner corners: `24 x 17`
- Square size: `0.0075 m`
- Captured samples used: `23`
- OpenCV RMS: `0.509367 px`
- Mean reprojection error: `0.441138 px`
- Undistort alpha: `1`
- Crop undistorted image: `true`

## Intrinsics

```text
K =
 1346.90065200     0.00000000   966.97858581
    0.00000000  1345.01523378   560.83557104
    0.00000000     0.00000000     1.00000000

D =
    0.16098872    -0.53274248     0.00175710     0.00084688     0.49992359
```

## Intrinsic Parameter Standard Deviations

```text
    0.22318994
    0.26024046
    0.32713272
    0.25071967
    0.00104036
    0.00470880
    0.00006891
    0.00008749
    0.00626099
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

- OpenCV per-view reprojection error mean: `0.501035 px`
- OpenCV per-view reprojection error median: `0.488029 px`
- OpenCV per-view reprojection error max: `0.692712 px`

## Per-Sample Checks

| Sample | Reproj mean (px) | Reproj max (px) | Raw border RMS (px) | Corrected border RMS (px) | Border improvement (px) | Improvement (%) | Raw lines | Corners | Corrected |
|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| 1 | 0.327803 | 0.854031 | 0.431054 | 0.147592 | 0.283462 | 65.7603 | `sample_0001.png` | `sample_0001.png` | `sample_0001.png` |
| 2 | 0.432932 | 1.30385 | 0.511231 | 0.234921 | 0.27631 | 54.0479 | `sample_0002.png` | `sample_0002.png` | `sample_0002.png` |
| 3 | 0.420284 | 1.3671 | 0.261363 | 0.263068 | -0.00170422 | -0.652052 | `sample_0003.png` | `sample_0003.png` | `sample_0003.png` |
| 4 | 0.453927 | 1.74224 | 0.623226 | 0.273995 | 0.34923 | 56.0359 | `sample_0004.png` | `sample_0004.png` | `sample_0004.png` |
| 5 | 0.40566 | 1.24862 | 0.715394 | 0.242021 | 0.473373 | 66.1695 | `sample_0005.png` | `sample_0005.png` | `sample_0005.png` |
| 6 | 0.399549 | 1.20879 | 0.625674 | 0.227419 | 0.398255 | 63.6521 | `sample_0006.png` | `sample_0006.png` | `sample_0006.png` |
| 7 | 0.403096 | 1.00629 | 0.404411 | 0.18808 | 0.216331 | 53.4929 | `sample_0007.png` | `sample_0007.png` | `sample_0007.png` |
| 8 | 0.461806 | 1.14118 | 0.555892 | 0.264753 | 0.291139 | 52.3733 | `sample_0008.png` | `sample_0008.png` | `sample_0008.png` |
| 9 | 0.418005 | 0.974105 | 0.624965 | 0.290272 | 0.334693 | 53.5539 | `sample_0009.png` | `sample_0009.png` | `sample_0009.png` |
| 10 | 0.416901 | 1.24253 | 0.564044 | 0.250061 | 0.313983 | 55.6663 | `sample_0010.png` | `sample_0010.png` | `sample_0010.png` |
| 11 | 0.463476 | 1.55209 | 0.573777 | 0.252498 | 0.32128 | 55.9938 | `sample_0011.png` | `sample_0011.png` | `sample_0011.png` |
| 12 | 0.430938 | 1.67368 | 0.446297 | 0.301202 | 0.145095 | 32.5108 | `sample_0012.png` | `sample_0012.png` | `sample_0012.png` |
| 13 | 0.473698 | 1.50551 | 0.369028 | 0.29913 | 0.0698977 | 18.941 | `sample_0013.png` | `sample_0013.png` | `sample_0013.png` |
| 14 | 0.481696 | 1.27204 | 0.376145 | 0.214807 | 0.161337 | 42.8924 | `sample_0014.png` | `sample_0014.png` | `sample_0014.png` |
| 15 | 0.404648 | 1.19126 | 0.463958 | 0.236942 | 0.227016 | 48.9302 | `sample_0015.png` | `sample_0015.png` | `sample_0015.png` |
| 16 | 0.605923 | 1.79016 | 0.451579 | 0.3758 | 0.075779 | 16.7809 | `sample_0016.png` | `sample_0016.png` | `sample_0016.png` |
| 17 | 0.588779 | 1.74979 | 0.58026 | 0.423254 | 0.157006 | 27.0579 | `sample_0017.png` | `sample_0017.png` | `sample_0017.png` |
| 18 | 0.371757 | 1.31992 | 0.224079 | 0.149564 | 0.0745151 | 33.254 | `sample_0018.png` | `sample_0018.png` | `sample_0018.png` |
| 19 | 0.293079 | 0.736618 | 0.187758 | 0.126253 | 0.0615049 | 32.7575 | `sample_0019.png` | `sample_0019.png` | `sample_0019.png` |
| 20 | 0.319188 | 0.886666 | 0.253027 | 0.100444 | 0.152584 | 60.3033 | `sample_0020.png` | `sample_0020.png` | `sample_0020.png` |
| 21 | 0.601969 | 2.03059 | 0.572742 | 0.468393 | 0.104349 | 18.2192 | `sample_0021.png` | `sample_0021.png` | `sample_0021.png` |
| 22 | 0.486986 | 1.55153 | 0.572903 | 0.360246 | 0.212657 | 37.1192 | `sample_0022.png` | `sample_0022.png` | `sample_0022.png` |
| 23 | 0.484083 | 1.551 | 0.569055 | 0.363044 | 0.206011 | 36.2023 | `sample_0023.png` | `sample_0023.png` | `sample_0023.png` |

## Aggregate Straightness Check

- Mean raw border RMS: `0.476429 px`
- Mean corrected border RMS: `0.263207 px`
- Mean border straightness improvement: `0.213222 px`
- Mean border straightness improvement: `42.6549 %`

## Qualitative Interpretation

The validation bundle is intended to answer two questions:

1. **Do the fitted parameters reproduce the observed corner locations well?**  
   This is summarized by the OpenCV RMS value, the mean reprojection error, and the per-sample reprojection statistics.

2. **Do straight physical chessboard borders become straighter after undistortion?**  
   This is summarized by the raw-versus-corrected border-line RMS residuals and the saved validation images.

Observed result: the corrected images reduce the mean border-line residual by `0.213222 px` on average (`42.6549 %` improvement).

Use the images in `01_uncorrected_with_lines`, `02_corner_detection`, and `03_corrected_image` together with the reprojection numbers above to judge calibration quality for this run.
