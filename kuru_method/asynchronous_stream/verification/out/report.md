# Async-stream verification report

Datasets root: `C:\Users\Christian Villanueva\Documents\College Files\Thesis\esp32_mock_webserver\kuru_method\asynchronous_stream\datasets_a3_ls`


## Per-dataset summary

| Dataset | First fail | Layers run |
|---|---|---|
| 0s | layer1_uwb_raw | 6 |
| 0s- | layer1_uwb_raw | 4 |
| 1s | layer1_uwb_raw | 5 |
| 1s- | layer1_uwb_raw | 4 |
| 2s | layer1_uwb_raw | 5 |
| 2s- | layer1_uwb_raw | 4 |
| 3s | layer1_uwb_raw | 5 |
| 3s- | layer1_uwb_raw | 4 |
| 4s | layer1_uwb_raw | 5 |
| 4s- | layer1_uwb_raw | 4 |
| ABC | -- | 2 |
| ABC- | -- | 1 |
| CIRCLE | layer6_ekf_end_to_end | 4 |
| CIRCLE- | -- | 2 |
| EastS | layer4_imu_integration | 4 |
| EastS- | -- | 1 |
| HELLO | -- | 2 |
| HELLO- | -- | 1 |
| NorthS | layer4_imu_integration | 4 |
| NorthS- | -- | 1 |
| SQUARE | layer6_ekf_end_to_end | 3 |
| SQUARE- | -- | 2 |
| STAR | -- | 1 |
| STAR- | layer0_stream_health | 1 |
| SouthS | layer4_imu_integration | 4 |
| SouthS- | -- | 1 |
| TRIANGLE | layer6_ekf_end_to_end | 3 |
| TRIANGLE- | -- | 2 |
| WestS | layer4_imu_integration | 4 |
| WestS- | -- | 1 |
| abcS | -- | 1 |
| abcS- | layer0_stream_health | 1 |
| circleS | -- | 1 |
| circleS- | -- | 1 |
| clockwiseM | -- | 1 |
| clockwiseM- | -- | 1 |
| corners | -- | 1 |
| corners- | -- | 1 |
| dline_A0_A2 | layer2_uwb_position | 3 |
| dline_A0_A2- | layer2_uwb_position | 2 |
| dline_A3_A1 | layer2_uwb_position | 3 |
| dline_A3_A1- | -- | 2 |
| helloS | -- | 1 |
| helloS- | layer0_stream_health | 1 |
| hline | layer4_imu_integration | 5 |
| hline- | -- | 2 |
| mix-mix_method | -- | 1 |
| mix-mix_method- | -- | 1 |
| revclockwiseM | -- | 1 |
| revclockwiseM- | -- | 1 |
| squareS | -- | 1 |
| squareS- | -- | 1 |
| starS | -- | 1 |
| starS- | layer0_stream_health | 1 |
| triangleS | -- | 1 |
| triangleS- | -- | 1 |
| vline | layer0_stream_health | 4 |
| vline- | layer0_stream_health | 2 |

## Detailed results

### [PASS] layer0_stream_health         0s-
- **imu_count**: 2919
- **uwb_count**: 291
- **duration_s**: 29.2
- **imu_rate_hz**: 100.3
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.171
- **uwb_loss_pct**: 0.3425
- **imu_dt_spike_ms**: 44.17
- **uwb_dt_spike_ms**: 200
![layer0_stream_health 0s-](out\layer0_stream_health\0s-.png)

### [PASS] layer0_stream_health         0s
- **imu_count**: 2106
- **uwb_count**: 210
- **duration_s**: 21.08
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2369
- **uwb_loss_pct**: 0.4739
- **imu_dt_spike_ms**: 48.98
- **uwb_dt_spike_ms**: 200
![layer0_stream_health 0s](out\layer0_stream_health\0s.png)

### [PASS] layer0_stream_health         1s-
- **imu_count**: 2368
- **uwb_count**: 237
- **duration_s**: 23.71
- **imu_rate_hz**: 100.3
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2527
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 51.68
- **uwb_dt_spike_ms**: 101
![layer0_stream_health 1s-](out\layer0_stream_health\1s-.png)

### [PASS] layer0_stream_health         1s
- **imu_count**: 2272
- **uwb_count**: 228
- **duration_s**: 22.75
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2634
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 49.98
- **uwb_dt_spike_ms**: 101
![layer0_stream_health 1s](out\layer0_stream_health\1s.png)

### [PASS] layer0_stream_health         2s-
- **imu_count**: 3105
- **uwb_count**: 311
- **duration_s**: 31.06
- **imu_rate_hz**: 100.3
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1608
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 52.81
- **uwb_dt_spike_ms**: 101
![layer0_stream_health 2s-](out\layer0_stream_health\2s-.png)

### [PASS] layer0_stream_health         2s
- **imu_count**: 2823
- **uwb_count**: 282
- **duration_s**: 28.25
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1768
- **uwb_loss_pct**: 0.3534
- **imu_dt_spike_ms**: 60.9
- **uwb_dt_spike_ms**: 200
![layer0_stream_health 2s](out\layer0_stream_health\2s.png)

### [PASS] layer0_stream_health         3s-
- **imu_count**: 3278
- **uwb_count**: 328
- **duration_s**: 32.87
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.395
- **uwb_loss_pct**: 0.304
- **imu_dt_spike_ms**: 60.9
- **uwb_dt_spike_ms**: 200
![layer0_stream_health 3s-](out\layer0_stream_health\3s-.png)

### [PASS] layer0_stream_health         3s
- **imu_count**: 3182
- **uwb_count**: 318
- **duration_s**: 31.83
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1569
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 39.06
- **uwb_dt_spike_ms**: 101
![layer0_stream_health 3s](out\layer0_stream_health\3s.png)

### [PASS] layer0_stream_health         4s-
- **imu_count**: 3564
- **uwb_count**: 357
- **duration_s**: 35.65
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1401
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 36.41
- **uwb_dt_spike_ms**: 101
![layer0_stream_health 4s-](out\layer0_stream_health\4s-.png)

### [PASS] layer0_stream_health         4s
- **imu_count**: 3637
- **uwb_count**: 363
- **duration_s**: 36.37
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1099
- **uwb_loss_pct**: 0.2747
- **imu_dt_spike_ms**: 42.04
- **uwb_dt_spike_ms**: 200
![layer0_stream_health 4s](out\layer0_stream_health\4s.png)

### [PASS] layer0_stream_health         ABC-
- **imu_count**: 2561
- **uwb_count**: 255
- **duration_s**: 25.66
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.3502
- **uwb_loss_pct**: 0.3906
- **imu_dt_spike_ms**: 50.08
- **uwb_dt_spike_ms**: 200
![layer0_stream_health ABC-](out\layer0_stream_health\ABC-.png)

### [PASS] layer0_stream_health         ABC
- **imu_count**: 2561
- **uwb_count**: 257
- **duration_s**: 25.76
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.6594
- **uwb_loss_pct**: 0.3876
- **imu_dt_spike_ms**: 50.92
- **uwb_dt_spike_ms**: 200
![layer0_stream_health ABC](out\layer0_stream_health\ABC.png)

### [FAIL] layer0_stream_health         abcS-
- **imu_count**: 1921
- **uwb_count**: 189
- **duration_s**: -4275
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 2.04
- **uwb_loss_pct**: 3.571
- **imu_dt_spike_ms**: 49.23
- **uwb_dt_spike_ms**: 600
![layer0_stream_health abcS-](out\layer0_stream_health\abcS-.png)

### [PASS] layer0_stream_health         abcS
- **imu_count**: 2012
- **uwb_count**: 201
- **duration_s**: 20.13
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1984
- **uwb_loss_pct**: 0.495
- **imu_dt_spike_ms**: 33.59
- **uwb_dt_spike_ms**: 200
![layer0_stream_health abcS](out\layer0_stream_health\abcS.png)

### [PASS] layer0_stream_health         CIRCLE-
- **imu_count**: 1818
- **uwb_count**: 182
- **duration_s**: 18.25
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.4926
- **uwb_loss_pct**: 0.5464
- **imu_dt_spike_ms**: 40.02
- **uwb_dt_spike_ms**: 200
![layer0_stream_health CIRCLE-](out\layer0_stream_health\CIRCLE-.png)

### [PASS] layer0_stream_health         CIRCLE
- **imu_count**: 1556
- **uwb_count**: 156
- **duration_s**: 15.58
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2564
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 50.49
- **uwb_dt_spike_ms**: 101
![layer0_stream_health CIRCLE](out\layer0_stream_health\CIRCLE.png)

### [PASS] layer0_stream_health         circleS-
- **imu_count**: 1364
- **uwb_count**: 136
- **duration_s**: 13.67
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.3652
- **uwb_loss_pct**: 0.7299
- **imu_dt_spike_ms**: 41.14
- **uwb_dt_spike_ms**: 200
![layer0_stream_health circleS-](out\layer0_stream_health\circleS-.png)

### [PASS] layer0_stream_health         circleS
- **imu_count**: 1363
- **uwb_count**: 137
- **duration_s**: 13.66
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.3655
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 42.8
- **uwb_dt_spike_ms**: 101
![layer0_stream_health circleS](out\layer0_stream_health\circleS.png)

### [PASS] layer0_stream_health         clockwiseM-
- **imu_count**: 3636
- **uwb_count**: 364
- **duration_s**: 36.39
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1373
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 54.41
- **uwb_dt_spike_ms**: 101
![layer0_stream_health clockwiseM-](out\layer0_stream_health\clockwiseM-.png)

### [PASS] layer0_stream_health         clockwiseM
- **imu_count**: 3017
- **uwb_count**: 303
- **duration_s**: 30.41
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.887
- **uwb_loss_pct**: 0.6557
- **imu_dt_spike_ms**: 62.44
- **uwb_dt_spike_ms**: 200
![layer0_stream_health clockwiseM](out\layer0_stream_health\clockwiseM.png)

### [PASS] layer0_stream_health         corners-
- **imu_count**: 3734
- **uwb_count**: 374
- **duration_s**: 37.65
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.9286
- **uwb_loss_pct**: 0.7958
- **imu_dt_spike_ms**: 75.21
- **uwb_dt_spike_ms**: 200
![layer0_stream_health corners-](out\layer0_stream_health\corners-.png)

### [PASS] layer0_stream_health         corners
- **imu_count**: 3637
- **uwb_count**: 363
- **duration_s**: 36.54
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.6013
- **uwb_loss_pct**: 0.5479
- **imu_dt_spike_ms**: 39.92
- **uwb_dt_spike_ms**: 200
![layer0_stream_health corners](out\layer0_stream_health\corners.png)

### [PASS] layer0_stream_health         dline_A0_A2-
- **imu_count**: 1751
- **uwb_count**: 176
- **duration_s**: 17.74
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 1.408
- **uwb_loss_pct**: 1.124
- **imu_dt_spike_ms**: 60.33
- **uwb_dt_spike_ms**: 200
![layer0_stream_health dline_A0_A2-](out\layer0_stream_health\dline_A0_A2-.png)

### [PASS] layer0_stream_health         dline_A0_A2
- **imu_count**: 1818
- **uwb_count**: 182
- **duration_s**: 18.24
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.4381
- **uwb_loss_pct**: 0.5464
- **imu_dt_spike_ms**: 36.04
- **uwb_dt_spike_ms**: 200
![layer0_stream_health dline_A0_A2](out\layer0_stream_health\dline_A0_A2.png)

### [PASS] layer0_stream_health         dline_A3_A1-
- **imu_count**: 1742
- **uwb_count**: 175
- **duration_s**: 17.61
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 1.247
- **uwb_loss_pct**: 0.5682
- **imu_dt_spike_ms**: 92.95
- **uwb_dt_spike_ms**: 200
![layer0_stream_health dline_A3_A1-](out\layer0_stream_health\dline_A3_A1-.png)

### [PASS] layer0_stream_health         dline_A3_A1
- **imu_count**: 1555
- **uwb_count**: 154
- **duration_s**: 15.78
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 1.582
- **uwb_loss_pct**: 0.6452
- **imu_dt_spike_ms**: 59.84
- **uwb_dt_spike_ms**: 200
![layer0_stream_health dline_A3_A1](out\layer0_stream_health\dline_A3_A1.png)

### [PASS] layer0_stream_health         EastS-
- **imu_count**: 2273
- **uwb_count**: 227
- **duration_s**: 22.76
- **imu_rate_hz**: 100.3
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2633
- **uwb_loss_pct**: 0.4386
- **imu_dt_spike_ms**: 50.8
- **uwb_dt_spike_ms**: 200
![layer0_stream_health EastS-](out\layer0_stream_health\EastS-.png)

### [PASS] layer0_stream_health         EastS
- **imu_count**: 2011
- **uwb_count**: 201
- **duration_s**: 20.13
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.248
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 49.09
- **uwb_dt_spike_ms**: 101
![layer0_stream_health EastS](out\layer0_stream_health\EastS.png)

### [PASS] layer0_stream_health         HELLO-
- **imu_count**: 3181
- **uwb_count**: 319
- **duration_s**: 31.88
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.3446
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 65.92
- **uwb_dt_spike_ms**: 101
![layer0_stream_health HELLO-](out\layer0_stream_health\HELLO-.png)

### [PASS] layer0_stream_health         HELLO
- **imu_count**: 3021
- **uwb_count**: 303
- **duration_s**: 30.35
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.5923
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 65.25
- **uwb_dt_spike_ms**: 101
![layer0_stream_health HELLO](out\layer0_stream_health\HELLO.png)

### [FAIL] layer0_stream_health         helloS-
- **imu_count**: 2208
- **uwb_count**: 219
- **duration_s**: 22.63
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 2.56
- **uwb_loss_pct**: 1.794
- **imu_dt_spike_ms**: 62.77
- **uwb_dt_spike_ms**: 200
![layer0_stream_health helloS-](out\layer0_stream_health\helloS-.png)

### [PASS] layer0_stream_health         helloS
- **imu_count**: 2013
- **uwb_count**: 200
- **duration_s**: 20.17
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2972
- **uwb_loss_pct**: 0.4975
- **imu_dt_spike_ms**: 33.03
- **uwb_dt_spike_ms**: 200
![layer0_stream_health helloS](out\layer0_stream_health\helloS.png)

### [PASS] layer0_stream_health         hline-
- **imu_count**: 1748
- **uwb_count**: 174
- **duration_s**: 17.51
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.3421
- **uwb_loss_pct**: 0.5714
- **imu_dt_spike_ms**: 31.95
- **uwb_dt_spike_ms**: 200
![layer0_stream_health hline-](out\layer0_stream_health\hline-.png)

### [PASS] layer0_stream_health         hline
- **imu_count**: 1557
- **uwb_count**: 155
- **duration_s**: 15.61
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.3839
- **uwb_loss_pct**: 0.641
- **imu_dt_spike_ms**: 56.31
- **uwb_dt_spike_ms**: 200
![layer0_stream_health hline](out\layer0_stream_health\hline.png)

### [PASS] layer0_stream_health         mix-mix_method-
- **imu_count**: 3010
- **uwb_count**: 301
- **duration_s**: 30.12
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1658
- **uwb_loss_pct**: 0.3311
- **imu_dt_spike_ms**: 39.93
- **uwb_dt_spike_ms**: 200
![layer0_stream_health mix-mix_method-](out\layer0_stream_health\mix-mix_method-.png)

### [PASS] layer0_stream_health         mix-mix_method
- **imu_count**: 2660
- **uwb_count**: 265
- **duration_s**: 26.61
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1876
- **uwb_loss_pct**: 0.3759
- **imu_dt_spike_ms**: 41.91
- **uwb_dt_spike_ms**: 200
![layer0_stream_health mix-mix_method](out\layer0_stream_health\mix-mix_method.png)

### [PASS] layer0_stream_health         NorthS-
- **imu_count**: 2727
- **uwb_count**: 273
- **duration_s**: 27.27
- **imu_rate_hz**: 100.3
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1465
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 30.95
- **uwb_dt_spike_ms**: 101
![layer0_stream_health NorthS-](out\layer0_stream_health\NorthS-.png)

### [PASS] layer0_stream_health         NorthS
- **imu_count**: 2369
- **uwb_count**: 237
- **duration_s**: 23.7
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1686
- **uwb_loss_pct**: 0.4202
- **imu_dt_spike_ms**: 32.21
- **uwb_dt_spike_ms**: 200
![layer0_stream_health NorthS](out\layer0_stream_health\NorthS.png)

### [PASS] layer0_stream_health         revclockwiseM-
- **imu_count**: 3277
- **uwb_count**: 328
- **duration_s**: 32.78
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1523
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 35.85
- **uwb_dt_spike_ms**: 101
![layer0_stream_health revclockwiseM-](out\layer0_stream_health\revclockwiseM-.png)

### [PASS] layer0_stream_health         revclockwiseM
- **imu_count**: 3278
- **uwb_count**: 327
- **duration_s**: 32.79
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1523
- **uwb_loss_pct**: 0.3049
- **imu_dt_spike_ms**: 39.3
- **uwb_dt_spike_ms**: 200
![layer0_stream_health revclockwiseM](out\layer0_stream_health\revclockwiseM.png)

### [PASS] layer0_stream_health         SouthS-
- **imu_count**: 2464
- **uwb_count**: 246
- **duration_s**: 24.66
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2025
- **uwb_loss_pct**: 0.4049
- **imu_dt_spike_ms**: 43.11
- **uwb_dt_spike_ms**: 200
![layer0_stream_health SouthS-](out\layer0_stream_health\SouthS-.png)

### [PASS] layer0_stream_health         SouthS
- **imu_count**: 2273
- **uwb_count**: 227
- **duration_s**: 22.76
- **imu_rate_hz**: 100.3
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2633
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 50.84
- **uwb_dt_spike_ms**: 101
![layer0_stream_health SouthS](out\layer0_stream_health\SouthS.png)

### [PASS] layer0_stream_health         SQUARE-
- **imu_count**: 2104
- **uwb_count**: 210
- **duration_s**: 21.05
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.1898
- **uwb_loss_pct**: 0.4739
- **imu_dt_spike_ms**: 31.82
- **uwb_dt_spike_ms**: 200
![layer0_stream_health SQUARE-](out\layer0_stream_health\SQUARE-.png)

### [PASS] layer0_stream_health         SQUARE
- **imu_count**: 2111
- **uwb_count**: 207
- **duration_s**: 21.14
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2834
- **uwb_loss_pct**: 1.896
- **imu_dt_spike_ms**: 50.97
- **uwb_dt_spike_ms**: 200
![layer0_stream_health SQUARE](out\layer0_stream_health\SQUARE.png)

### [PASS] layer0_stream_health         squareS-
- **imu_count**: 1650
- **uwb_count**: 164
- **duration_s**: 16.55
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.4825
- **uwb_loss_pct**: 0.6061
- **imu_dt_spike_ms**: 41.86
- **uwb_dt_spike_ms**: 200
![layer0_stream_health squareS-](out\layer0_stream_health\squareS-.png)

### [PASS] layer0_stream_health         squareS
- **imu_count**: 1652
- **uwb_count**: 166
- **duration_s**: 16.59
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.5418
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 57.13
- **uwb_dt_spike_ms**: 101
![layer0_stream_health squareS](out\layer0_stream_health\squareS.png)

### [FAIL] layer0_stream_health         STAR-
- **imu_count**: 2007
- **uwb_count**: 201
- **duration_s**: 20.55
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 2.431
- **uwb_loss_pct**: 2.427
- **imu_dt_spike_ms**: 315.2
- **uwb_dt_spike_ms**: 500
![layer0_stream_health STAR-](out\layer0_stream_health\STAR-.png)

### [PASS] layer0_stream_health         STAR
- **imu_count**: 2106
- **uwb_count**: 211
- **duration_s**: 21.13
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.4726
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 49.35
- **uwb_dt_spike_ms**: 101
![layer0_stream_health STAR](out\layer0_stream_health\STAR.png)

### [FAIL] layer0_stream_health         starS-
- **imu_count**: 1565
- **uwb_count**: 147
- **duration_s**: 18.25
- **imu_rate_hz**: 99.89
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 14.39
- **uwb_loss_pct**: 19.23
- **imu_dt_spike_ms**: 150
- **uwb_dt_spike_ms**: 800
![layer0_stream_health starS-](out\layer0_stream_health\starS-.png)

### [PASS] layer0_stream_health         starS
- **imu_count**: 1556
- **uwb_count**: 156
- **duration_s**: 15.84
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 1.953
- **uwb_loss_pct**: 1.887
- **imu_dt_spike_ms**: 110.9
- **uwb_dt_spike_ms**: 400
![layer0_stream_health starS](out\layer0_stream_health\starS.png)

### [PASS] layer0_stream_health         TRIANGLE-
- **imu_count**: 2011
- **uwb_count**: 199
- **duration_s**: 20.15
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2975
- **uwb_loss_pct**: 0.995
- **imu_dt_spike_ms**: 46.64
- **uwb_dt_spike_ms**: 200
![layer0_stream_health TRIANGLE-](out\layer0_stream_health\TRIANGLE-.png)

### [PASS] layer0_stream_health         TRIANGLE
- **imu_count**: 1818
- **uwb_count**: 182
- **duration_s**: 18.21
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.3289
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 31.52
- **uwb_dt_spike_ms**: 101
![layer0_stream_health TRIANGLE](out\layer0_stream_health\TRIANGLE.png)

### [PASS] layer0_stream_health         triangleS-
- **imu_count**: 1556
- **uwb_count**: 155
- **duration_s**: 15.6
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.3841
- **uwb_loss_pct**: 0.641
- **imu_dt_spike_ms**: 39.71
- **uwb_dt_spike_ms**: 200
![layer0_stream_health triangleS-](out\layer0_stream_health\triangleS-.png)

### [PASS] layer0_stream_health         triangleS
- **imu_count**: 1460
- **uwb_count**: 146
- **duration_s**: 14.65
- **imu_rate_hz**: 100.2
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.4772
- **uwb_loss_pct**: 0
- **imu_dt_spike_ms**: 59.32
- **uwb_dt_spike_ms**: 101
![layer0_stream_health triangleS](out\layer0_stream_health\triangleS.png)

### [FAIL] layer0_stream_health         vline-
- **imu_count**: 1746
- **uwb_count**: 167
- **duration_s**: 19.87
- **imu_rate_hz**: 99.9
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 12.26
- **uwb_loss_pct**: 16.08
- **imu_dt_spike_ms**: 101
- **uwb_dt_spike_ms**: 700
![layer0_stream_health vline-](out\layer0_stream_health\vline-.png)

### [FAIL] layer0_stream_health         vline
- **imu_count**: 1556
- **uwb_count**: 152
- **duration_s**: 16.22
- **imu_rate_hz**: 100.1
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 4.246
- **uwb_loss_pct**: 6.173
- **imu_dt_spike_ms**: 79.79
- **uwb_dt_spike_ms**: 800
![layer0_stream_health vline](out\layer0_stream_health\vline.png)

### [PASS] layer0_stream_health         WestS-
- **imu_count**: 2368
- **uwb_count**: 237
- **duration_s**: 23.72
- **imu_rate_hz**: 100.3
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2527
- **uwb_loss_pct**: 0.4202
- **imu_dt_spike_ms**: 50.93
- **uwb_dt_spike_ms**: 200
![layer0_stream_health WestS-](out\layer0_stream_health\WestS-.png)

### [PASS] layer0_stream_health         WestS
- **imu_count**: 1819
- **uwb_count**: 181
- **duration_s**: 18.2
- **imu_rate_hz**: 100.3
- **uwb_rate_hz**: 10
- **imu_loss_pct**: 0.2194
- **uwb_loss_pct**: 0.5495
- **imu_dt_spike_ms**: 31.23
- **uwb_dt_spike_ms**: 200
![layer0_stream_health WestS](out\layer0_stream_health\WestS.png)

### [FAIL] layer1_uwb_raw               0s
- **uwb_count**: 210
- **raw_mean**: [1.0094761904761902, 0.9854761904761901, 1.1160000000000005, 1.0260952380952386]
- **raw_std**: [0.02680106438472541, 0.031014772283111256, 0.023384976862446144, 0.03239621489046254]
- **cal_mean**: [0.8510761904761907, 0.9426761904761901, 0.9187000000000007, 0.8287952380952377]
- **cal_std**: [0.026801064384725393, 0.031014772283111256, 0.023384976862446085, 0.03239621489046255]
- **des_mean**: [0.8524333333333336, 0.9437476190476188, 0.9182000000000013, 0.828604761904762]
- **des_std**: [0.02243288048789384, 0.028894061328331003, 0.018742935176957107, 0.024696066318545127]
- **expected**: [0.894776508408664, 0.894776508408664, 0.894776508408664, 0.894776508408664]
- **raw_bias**: [0.11469968206752623, 0.0906996820675261, 0.22122349159133659, 0.13131872968657465]
- **cal_bias**: [-0.04370031793247331, 0.04789968206752615, 0.023923491591336776, -0.06598127031342627]
- **des_bias**: [-0.042343175075330364, 0.04897111063895487, 0.023423491591337386, -0.06617174650390201]
- **truth**: (0.625, 0.62)
- **max_cal_abs_bias**: 0.06598
- **max_cal_std**: 0.0324
![layer1_uwb_raw 0s](out\layer1_uwb_raw\0s.png)

### [FAIL] layer1_uwb_raw               0s-
- **uwb_count**: 291
- **raw_mean**: [0.9869415807560135, 0.9072164948453595, 1.114639175257731, 1.042920962199313]
- **raw_std**: [0.02836785890417269, 0.026635206581483235, 0.02276709458590123, 0.025553473488385013]
- **cal_mean**: [0.8285415807560126, 0.86441649484536, 0.9173391752577337, 0.8456209621993106]
- **cal_std**: [0.028367858904172785, 0.02663520658148325, 0.022767094585901246, 0.02555347348838501]
- **des_mean**: [0.8325450171821299, 0.8648632302405488, 0.9175453608247441, 0.8463082474226779]
- **des_std**: [0.01733688044675671, 0.02363896519469913, 0.017277604163361777, 0.017636199515615278]
- **expected**: [0.894776508408664, 0.894776508408664, 0.894776508408664, 0.894776508408664]
- **raw_bias**: [0.0921650723473495, 0.012439986436695527, 0.21986266684906697, 0.148144453790649]
- **cal_bias**: [-0.06623492765265138, -0.030360013563303978, 0.022562666849069712, -0.04915554620935336]
- **des_bias**: [-0.06223149122653404, -0.029913278168115176, 0.022768852416080176, -0.04846826098598611]
- **truth**: (0.625, 0.62)
- **max_cal_abs_bias**: 0.06623
- **max_cal_std**: 0.02837
![layer1_uwb_raw 0s-](out\layer1_uwb_raw\0s-.png)

### [FAIL] layer1_uwb_raw               1s
- **uwb_count**: 228
- **raw_mean**: [0.3270175438596495, 1.244780701754387, 1.9040789473684203, 1.37140350877193]
- **raw_std**: [0.02488322991111644, 0.02005355057678347, 0.062495844737216526, 0.04991883224044727]
- **cal_mean**: [0.16861754385964917, 1.2019807017543864, 1.7067789473684207, 1.174103508771929]
- **cal_std**: [0.024883229911116404, 0.02005355057678344, 0.06249584473721656, 0.04991883224044727]
- **des_mean**: [0.16988947368421045, 1.2009500000000002, 1.7041912280701756, 1.1732921052631562]
- **des_std**: [0.018092464111273736, 0.013182050195945295, 0.05539701138528738, 0.04515300818549567]
- **expected**: [0.16278820596099705, 1.230447073221762, 1.7468829382646107, 1.2506398362438325]
- **raw_bias**: [0.16422933789865243, 0.014333628532624987, 0.1571960091038096, 0.12076367252809761]
- **cal_bias**: [0.005829337898652115, -0.02846637146737563, -0.04010399089618999, -0.07653632747190353]
- **des_bias**: [0.0071012677232134, -0.02949707322176187, -0.04269171019443507, -0.07734773098067627]
- **truth**: (0.03, 0.0)
- **max_cal_abs_bias**: 0.07654
- **max_cal_std**: 0.0625
![layer1_uwb_raw 1s](out\layer1_uwb_raw\1s.png)

### [FAIL] layer1_uwb_raw               1s-
- **uwb_count**: 237
- **raw_mean**: [0.3301265822784813, 1.2621940928270061, 2.003164556962026, 1.3663713080168787]
- **raw_std**: [0.029574778464606787, 0.01885533106071346, 0.09767176976368327, 0.053638866068751054]
- **cal_mean**: [0.17172658227848106, 1.2193940928270062, 1.8058645569620246, 1.1690713080168762]
- **cal_std**: [0.029574778464606814, 0.01885533106071346, 0.0976717697636833, 0.05363886606875099]
- **des_mean**: [0.17286582278481022, 1.220090295358652, 1.801413080168776, 1.1685227848101256]
- **des_std**: [0.020604805998363524, 0.01591713081868908, 0.08725133416825925, 0.04970650387289294]
- **expected**: [0.16278820596099705, 1.230447073221762, 1.7468829382646107, 1.2506398362438325]
- **raw_bias**: [0.16733837631748424, 0.031747019605244065, 0.2562816186974155, 0.11573147177304621]
- **cal_bias**: [0.00893837631748401, -0.011052980394755885, 0.05898161869741392, -0.08156852822695626]
- **des_bias**: [0.010077616823813168, -0.010356777863109956, 0.054530141904165275, -0.08211705143370684]
- **truth**: (0.03, 0.0)
- **max_cal_abs_bias**: 0.08157
- **max_cal_std**: 0.09767
![layer1_uwb_raw 1s-](out\layer1_uwb_raw\1s-.png)

### [FAIL] layer1_uwb_raw               2s
- **uwb_count**: 282
- **raw_mean**: [1.3707092198581547, 0.28964539007092194, 1.3012411347517734, 2.037695035460994]
- **raw_std**: [0.023689641931704405, 0.03849862017455041, 0.02519889694456837, 0.04424311354234379]
- **cal_mean**: [1.2123092198581549, 0.24684539007092168, 1.1039411347517742, 1.8403950354609957]
- **cal_std**: [0.023689641931704447, 0.03849862017455037, 0.025198896944568394, 0.04424311354234384]
- **des_mean**: [1.214578723404253, 0.24923900709219857, 1.1038524822695048, 1.8392248226950385]
- **des_std**: [0.019415015058397923, 0.027563760443638268, 0.020277371245269456, 0.038472350126543445]
- **expected**: [1.230447073221762, 0.16278820596099705, 1.2506398362438325, 1.7468829382646107]
- **raw_bias**: [0.1402621466363927, 0.1268571841099249, 0.05060129850794093, 0.29081209719638323]
- **cal_bias**: [-0.01813785336360718, 0.08405718410992463, -0.14669870149205821, 0.09351209719638498]
- **des_bias**: [-0.015868349817508953, 0.08645080113120152, -0.14678735397432763, 0.09234188443042779]
- **truth**: (1.22, 0.0)
- **max_cal_abs_bias**: 0.1467
- **max_cal_std**: 0.04424
![layer1_uwb_raw 2s](out\layer1_uwb_raw\2s.png)

### [FAIL] layer1_uwb_raw               2s-
- **uwb_count**: 311
- **raw_mean**: [1.3806109324758848, 0.27913183279742837, 1.3491961414791003, 1.77922829581994]
- **raw_std**: [0.022822108282577143, 0.01850224587526157, 0.024197674370048196, 0.06045951559277605]
- **cal_mean**: [1.2222109324758854, 0.2363318327974279, 1.1518961414790971, 1.5819282958199352]
- **cal_std**: [0.022822108282577122, 0.01850224587526158, 0.024197674370048255, 0.06045951559277603]
- **des_mean**: [1.220538906752414, 0.23748938906752454, 1.1522176848874568, 1.5820569131832787]
- **des_std**: [0.014584253415221718, 0.0136885375786295, 0.01974334131987421, 0.05750077261151997]
- **expected**: [1.230447073221762, 0.16278820596099705, 1.2506398362438325, 1.7468829382646107]
- **raw_bias**: [0.1501638592541228, 0.11634362683643132, 0.09855630523526782, 0.03234535755532919]
- **cal_bias**: [-0.008236140745876641, 0.07354362683643084, -0.09874369476473532, -0.1649546424446755]
- **des_bias**: [-0.009908166469348112, 0.07470118310652749, -0.09842215135637566, -0.16482602508133204]
- **truth**: (1.22, 0.0)
- **max_cal_abs_bias**: 0.165
- **max_cal_std**: 0.06046
![layer1_uwb_raw 2s-](out\layer1_uwb_raw\2s-.png)

### [FAIL] layer1_uwb_raw               3s
- **uwb_count**: 318
- **raw_mean**: [1.8553459119496862, 1.2386477987421354, 0.5169182389937104, 1.3635849056603762]
- **raw_std**: [0.04861321965059791, 0.1202345526559946, 0.037491293505596185, 0.028078573913954574]
- **cal_mean**: [1.6969459119496855, 1.195847798742136, 0.31961823899371106, 1.1662849056603757]
- **cal_std**: [0.04861321965059792, 0.12023455265599464, 0.037491293505596185, 0.028078573913954564]
- **des_mean**: [1.696395597484276, 1.1948572327044007, 0.3190679245283026, 1.1662691823899345]
- **des_std**: [0.03674712643694156, 0.11486770676630943, 0.02654828002756445, 0.023703751924671203]
- **expected**: [1.7468829382646107, 1.2506398362438325, 0.16278820596099705, 1.230447073221762]
- **raw_bias**: [0.10846297368507551, -0.011992037501697084, 0.3541300330327133, 0.1331378324386141]
- **cal_bias**: [-0.04993702631492525, -0.05479203750169637, 0.156830033032714, -0.06416216756138637]
- **des_bias**: [-0.05048734078033479, -0.05578260353943176, 0.15627971856730558, -0.06417789083182757]
- **truth**: (1.22, 1.24)
- **max_cal_abs_bias**: 0.1568
- **max_cal_std**: 0.1202
![layer1_uwb_raw 3s](out\layer1_uwb_raw\3s.png)

### [FAIL] layer1_uwb_raw               3s-
- **uwb_count**: 328
- **raw_mean**: [1.9177134146341475, 1.4234756097561003, 0.5319512195121944, 1.385670731707314]
- **raw_std**: [0.0668859846455056, 0.16582239438111712, 0.03655730631288709, 0.030913551720117873]
- **cal_mean**: [1.7593134146341454, 1.3806756097561004, 0.33465121951219556, 1.1883707317073169]
- **cal_std**: [0.0668859846455056, 0.16582239438111712, 0.03655730631288705, 0.030913551720117904]
- **des_mean**: [1.7571640243902413, 1.3876115853658573, 0.3339957317073177, 1.1884926829268299]
- **des_std**: [0.05086060144086215, 0.15474772384920893, 0.029320262390703913, 0.02456908517752415]
- **expected**: [1.7468829382646107, 1.2506398362438325, 0.16278820596099705, 1.230447073221762]
- **raw_bias**: [0.17083047636953674, 0.17283577351226787, 0.3691630135511973, 0.15522365848555197]
- **cal_bias**: [0.012430476369534649, 0.13003577351226792, 0.1718630135511985, -0.04207634151444517]
- **des_bias**: [0.010281086125630612, 0.1369717491220248, 0.17120752574632067, -0.0419543902949322]
- **truth**: (1.22, 1.24)
- **max_cal_abs_bias**: 0.1719
- **max_cal_std**: 0.1658
![layer1_uwb_raw 3s-](out\layer1_uwb_raw\3s-.png)

### [FAIL] layer1_uwb_raw               4s
- **uwb_count**: 363
- **raw_mean**: [1.345206611570248, 1.5299724517906368, 1.505619834710743, 0.4171074380165286]
- **raw_std**: [0.19670466671307102, 0.21048350297878488, 0.12959074739517865, 0.2879741284384937]
- **cal_mean**: [1.1868066115702463, 1.4871724517906344, 1.3083198347107428, 0.219807438016529]
- **cal_std**: [0.19670466671307096, 0.21048350297878485, 0.12959074739517862, 0.2879741284384937]
- **des_mean**: [1.1874677685950392, 1.4896242424242432, 1.3159366558881227, 0.22451819318299807]
- **des_std**: [0.19395408315270096, 0.20647952070400624, 0.06294981314544572, 0.2874932821698536]
- **expected**: [1.2506398362438325, 1.7468829382646107, 1.230447073221762, 0.16278820596099705]
- **raw_bias**: [0.09456677532641544, -0.21691048647397393, 0.27517276148898095, 0.25431923205553153]
- **cal_bias**: [-0.06383322467358621, -0.2597104864739763, 0.0778727614889807, 0.05701923205553194]
- **des_bias**: [-0.06317206764879324, -0.25725869584036754, 0.08548958266636064, 0.061729987222001015]
- **truth**: (0.03, 1.24)
- **max_cal_abs_bias**: 0.2597
- **max_cal_std**: 0.288
![layer1_uwb_raw 4s](out\layer1_uwb_raw\4s.png)

### [FAIL] layer1_uwb_raw               4s-
- **uwb_count**: 357
- **raw_mean**: [1.3300000000000007, 1.5320168067226905, 1.5013165266106443, 0.37187675070027987]
- **raw_std**: [0.1032741314099876, 0.21744858627435623, 0.056013234542900435, 0.285066420964362]
- **cal_mean**: [1.1716, 1.4892168067226916, 1.304016526610643, 0.17457675070028]
- **cal_std**: [0.10327413140998759, 0.21744858627435623, 0.056013234542900456, 0.28506642096436197]
- **des_mean**: [1.1710117647058818, 1.4921019607843173, 1.30379243697479, 0.17419877694878516]
- **des_std**: [0.09846122060198129, 0.212682754293211, 0.045388887618791014, 0.2760473440368671]
- **expected**: [1.2506398362438325, 1.7468829382646107, 1.230447073221762, 0.16278820596099705]
- **raw_bias**: [0.07936016375616828, -0.21486613154192025, 0.27086945338888224, 0.20908854473928282]
- **cal_bias**: [-0.07903983624383248, -0.2576661315419191, 0.07356945338888088, 0.011788544739282952]
- **des_bias**: [-0.0796280715379507, -0.2547809774802934, 0.07334536375302791, 0.01141057098778811]
- **truth**: (0.03, 1.24)
- **max_cal_abs_bias**: 0.2577
- **max_cal_std**: 0.2851
![layer1_uwb_raw 4s-](out\layer1_uwb_raw\4s-.png)

### [FAIL] layer2_uwb_position          0s
- **n_points**: 210
- **kind**: stationary
- **mean**: [0.5609411826325599, 0.6375852362858996]
- **std**: [0.014275602769018726, 0.023737323460415904]
- **r95_m**: 0.0457
- **truth**: (0.625, 0.62)
- **bias_m**: 0.06643
![layer2_uwb_position 0s](out\layer2_uwb_position\0s.png)

### [PASS] layer2_uwb_position          0s-
- **n_points**: 291
- **kind**: stationary
- **mean**: [0.5894400019039004, 0.5971750978763073]
- **std**: [0.012373805092025443, 0.013297402133111832]
- **r95_m**: 0.03306
- **truth**: (0.625, 0.62)
- **bias_m**: 0.04226
![layer2_uwb_position 0s-](out\layer2_uwb_position\0s-.png)

### [FAIL] layer2_uwb_position          1s
- **n_points**: 228
- **kind**: stationary
- **mean**: [0.04873146928168844, 0.06250264108840381]
- **std**: [0.020008578742425693, 0.0343288642867463]
- **r95_m**: 0.07081
- **truth**: (0.03, 0.0)
- **bias_m**: 0.06525
![layer2_uwb_position 1s](out\layer2_uwb_position\1s.png)

### [FAIL] layer2_uwb_position          1s-
- **n_points**: 237
- **kind**: stationary
- **mean**: [0.01756392530808213, 0.061269056276643315]
- **std**: [0.03503251156741061, 0.03515967318632053]
- **r95_m**: 0.09888
- **truth**: (0.03, 0.0)
- **bias_m**: 0.06252
![layer2_uwb_position 1s-](out\layer2_uwb_position\1s-.png)

### [FAIL] layer2_uwb_position          2s
- **n_points**: 282
- **kind**: stationary
- **mean**: [1.286299719222756, 0.13971618582937878]
- **std**: [0.09185804756560541, 0.019701717821769396]
- **r95_m**: 0.1538
- **truth**: (1.22, 0.0)
- **bias_m**: 0.1546
![layer2_uwb_position 2s](out\layer2_uwb_position\2s.png)

### [FAIL] layer2_uwb_position          2s-
- **n_points**: 311
- **kind**: stationary
- **mean**: [1.1728962263786376, 0.1381240248263574]
- **std**: [0.023331931353535864, 0.013911382715114887]
- **r95_m**: 0.05084
- **truth**: (1.22, 0.0)
- **bias_m**: 0.1459
![layer2_uwb_position 2s-](out\layer2_uwb_position\2s-.png)

### [FAIL] layer2_uwb_position          3s
- **n_points**: 318
- **kind**: stationary
- **mean**: [1.1298959238586714, 1.1794835242540032]
- **std**: [0.05453823348371004, 0.11114946227924574]
- **r95_m**: 0.2622
- **truth**: (1.22, 1.24)
- **bias_m**: 0.1085
![layer2_uwb_position 3s](out\layer2_uwb_position\3s.png)

### [FAIL] layer2_uwb_position          3s-
- **n_points**: 328
- **kind**: stationary
- **mean**: [1.1106579930381875, 1.3832591648725447]
- **std**: [0.0359245836480822, 0.13586076329436672]
- **r95_m**: 0.3703
- **truth**: (1.22, 1.24)
- **bias_m**: 0.1802
![layer2_uwb_position 3s-](out\layer2_uwb_position\3s-.png)

### [FAIL] layer2_uwb_position          4s
- **n_points**: 363
- **kind**: stationary
- **mean**: [0.005275498808111473, 1.1295172349131954]
- **std**: [0.17017814772277315, 0.26508106629362216]
- **r95_m**: 0.8675
- **truth**: (0.03, 1.24)
- **bias_m**: 0.1132
![layer2_uwb_position 4s](out\layer2_uwb_position\4s.png)

### [FAIL] layer2_uwb_position          4s-
- **n_points**: 357
- **kind**: stationary
- **mean**: [0.03524578764519297, 1.0871780528656867]
- **std**: [0.18160501857117126, 0.25046134070479004]
- **r95_m**: 0.8932
- **truth**: (0.03, 1.24)
- **bias_m**: 0.1529
![layer2_uwb_position 4s-](out\layer2_uwb_position\4s-.png)

### [PASS] layer2_uwb_position          hline
- **n_points**: 155
- **kind**: line_h
- **angle_deg**: 2.12
- **expected_deg**: 0
- **angle_err_deg**: 2.12
- **span_m**: 1.343
- **rms_perp_m**: 0.04091
![layer2_uwb_position hline](out\layer2_uwb_position\hline.png)

### [PASS] layer2_uwb_position          hline-
- **n_points**: 174
- **kind**: line_h
- **angle_deg**: 0.7003
- **expected_deg**: 0
- **angle_err_deg**: 0.7003
- **span_m**: 1.34
- **rms_perp_m**: 0.03444
![layer2_uwb_position hline-](out\layer2_uwb_position\hline-.png)

### [PASS] layer2_uwb_position          vline
- **n_points**: 152
- **kind**: line_v
- **angle_deg**: -89.18
- **expected_deg**: 90
- **angle_err_deg**: 0.8162
- **span_m**: 0.9954
- **rms_perp_m**: 0.02892
![layer2_uwb_position vline](out\layer2_uwb_position\vline.png)

### [PASS] layer2_uwb_position          vline-
- **n_points**: 167
- **kind**: line_v
- **angle_deg**: -88.83
- **expected_deg**: 90
- **angle_err_deg**: 1.168
- **span_m**: 1.16
- **rms_perp_m**: 0.02352
![layer2_uwb_position vline-](out\layer2_uwb_position\vline-.png)

### [FAIL] layer2_uwb_position          dline_A0_A2
- **n_points**: 182
- **kind**: line_d02
- **angle_deg**: 44.04
- **expected_deg**: 44.77
- **angle_err_deg**: 0.7264
- **span_m**: 1.942
- **rms_perp_m**: 0.09022
![layer2_uwb_position dline_A0_A2](out\layer2_uwb_position\dline_A0_A2.png)

### [FAIL] layer2_uwb_position          dline_A0_A2-
- **n_points**: 176
- **kind**: line_d02
- **angle_deg**: 41.41
- **expected_deg**: 44.77
- **angle_err_deg**: 3.365
- **span_m**: 1.633
- **rms_perp_m**: 0.07256
![layer2_uwb_position dline_A0_A2-](out\layer2_uwb_position\dline_A0_A2-.png)

### [FAIL] layer2_uwb_position          dline_A3_A1
- **n_points**: 154
- **kind**: line_d31
- **angle_deg**: -38.29
- **expected_deg**: -44.77
- **angle_err_deg**: 6.475
- **span_m**: 1.752
- **rms_perp_m**: 0.05425
![layer2_uwb_position dline_A3_A1](out\layer2_uwb_position\dline_A3_A1.png)

### [PASS] layer2_uwb_position          dline_A3_A1-
- **n_points**: 175
- **kind**: line_d31
- **angle_deg**: -37.71
- **expected_deg**: -44.77
- **angle_err_deg**: 7.057
- **span_m**: 1.837
- **rms_perp_m**: 0.04704
![layer2_uwb_position dline_A3_A1-](out\layer2_uwb_position\dline_A3_A1-.png)

### [PASS] layer2_uwb_position          CIRCLE
- **n_points**: 156
- **kind**: shape_circle
- **center**: [0.5718727998429064, 0.5464810494807754]
- **radius_m**: 0.2067
- **rms_radial_m**: 0.03769
![layer2_uwb_position CIRCLE](out\layer2_uwb_position\CIRCLE.png)

### [PASS] layer2_uwb_position          CIRCLE-
- **n_points**: 182
- **kind**: shape_circle
- **center**: [0.563499732046805, 0.5866543349235513]
- **radius_m**: 0.1769
- **rms_radial_m**: 0.0401
![layer2_uwb_position CIRCLE-](out\layer2_uwb_position\CIRCLE-.png)

### [PASS] layer2_uwb_position          SQUARE
- **n_points**: 207
- **kind**: shape_square
- **hull_area_m2**: 0.355
- **bbox_w**: 0.7195
- **bbox_h**: 0.5847
![layer2_uwb_position SQUARE](out\layer2_uwb_position\SQUARE.png)

### [PASS] layer2_uwb_position          SQUARE-
- **n_points**: 210
- **kind**: shape_square
- **hull_area_m2**: 0.2729
- **bbox_w**: 0.6208
- **bbox_h**: 0.5872
![layer2_uwb_position SQUARE-](out\layer2_uwb_position\SQUARE-.png)

### [PASS] layer2_uwb_position          TRIANGLE
- **n_points**: 182
- **kind**: shape_triangle
- **hull_area_m2**: 0.1759
- **bbox_w**: 0.4877
- **bbox_h**: 0.5098
![layer2_uwb_position TRIANGLE](out\layer2_uwb_position\TRIANGLE.png)

### [PASS] layer2_uwb_position          TRIANGLE-
- **n_points**: 199
- **kind**: shape_triangle
- **hull_area_m2**: 0.2011
- **bbox_w**: 0.6165
- **bbox_h**: 0.4533
![layer2_uwb_position TRIANGLE-](out\layer2_uwb_position\TRIANGLE-.png)

### [FAIL] layer3_imu_raw               0s
- **imu_count**: 2106
- **duration_s**: 21.08
- **quat_drift_deg**: 5.637
- **acc_mean_xyz**: [5.3371320037988455e-05, -0.0030971984805318142, 0.004017378917378921]
- **acc_std_xyz**: [0.0801209641660075, 0.14824517495094058, 0.07596182346974599]
- **acc_noise_max_std**: 0.1482
- **deadband_ratio**: 1.853
- **above_deadband_pct**: 80.1
![layer3_imu_raw 0s](out\layer3_imu_raw\0s.png)

### [FAIL] layer3_imu_raw               0s-
- **imu_count**: 2919
- **duration_s**: 29.2
- **quat_drift_deg**: 1.822
- **acc_mean_xyz**: [-0.0013421377183967136, -0.0005359712230215812, -0.0006851318944844079]
- **acc_std_xyz**: [0.017744737525613223, 0.13767710281528286, 0.07558398484397051]
- **acc_noise_max_std**: 0.1377
- **deadband_ratio**: 1.721
- **above_deadband_pct**: 72.59
![layer3_imu_raw 0s-](out\layer3_imu_raw\0s-.png)

### [FAIL] layer3_imu_raw               1s
- **imu_count**: 2272
- **duration_s**: 22.75
- **quat_drift_deg**: 3.608
- **acc_mean_xyz**: [-0.0004017605633802806, 0.0009367517605633801, -4.339788732394342e-05]
- **acc_std_xyz**: [0.023730536065812814, 0.1203405283910115, 0.10447535721169188]
- **acc_noise_max_std**: 0.1203
- **deadband_ratio**: 1.504
- **above_deadband_pct**: 76.89
![layer3_imu_raw 1s](out\layer3_imu_raw\1s.png)

### [FAIL] layer3_imu_raw               1s-
- **imu_count**: 2368
- **duration_s**: 23.71
- **quat_drift_deg**: 6.933
- **acc_mean_xyz**: [0.004073395270270292, -0.0002875844594594591, -0.0008818412162162187]
- **acc_std_xyz**: [0.027051152463819263, 0.16077714893448974, 0.10535108405618179]
- **acc_noise_max_std**: 0.1608
- **deadband_ratio**: 2.01
- **above_deadband_pct**: 78.67
![layer3_imu_raw 1s-](out\layer3_imu_raw\1s-.png)

### [FAIL] layer3_imu_raw               2s
- **imu_count**: 2823
- **duration_s**: 28.24
- **quat_drift_deg**: 3.58
- **acc_mean_xyz**: [0.0031699964576691222, 0.00017314913212894143, -1.8313850513637883e-05]
- **acc_std_xyz**: [0.02064360575639718, 0.12332755265680119, 0.0819921642126154]
- **acc_noise_max_std**: 0.1233
- **deadband_ratio**: 1.542
- **above_deadband_pct**: 72.58
![layer3_imu_raw 2s](out\layer3_imu_raw\2s.png)

### [FAIL] layer3_imu_raw               2s-
- **imu_count**: 3105
- **duration_s**: 31.06
- **quat_drift_deg**: 1.924
- **acc_mean_xyz**: [0.003700386473429964, 0.0004379710144927544, -0.0002781320450885662]
- **acc_std_xyz**: [0.015286996142420036, 0.12078115052895202, 0.09538841134738078]
- **acc_noise_max_std**: 0.1208
- **deadband_ratio**: 1.51
- **above_deadband_pct**: 69.76
![layer3_imu_raw 2s-](out\layer3_imu_raw\2s-.png)

### [FAIL] layer3_imu_raw               3s
- **imu_count**: 3182
- **duration_s**: 31.83
- **quat_drift_deg**: 34.81
- **acc_mean_xyz**: [-0.0027744814582023936, -0.00033918918918919236, -0.0024778755499685508]
- **acc_std_xyz**: [0.3206352819895762, 0.27756950469769426, 0.2782432328025055]
- **acc_noise_max_std**: 0.3206
- **deadband_ratio**: 4.008
- **above_deadband_pct**: 75.46
![layer3_imu_raw 3s](out\layer3_imu_raw\3s.png)

### [FAIL] layer3_imu_raw               3s-
- **imu_count**: 3278
- **duration_s**: 32.87
- **quat_drift_deg**: 35.2
- **acc_mean_xyz**: [-0.008824832214765082, 0.003917236119585114, -0.004928187919463078]
- **acc_std_xyz**: [0.18134523491886537, 0.4260664966548516, 0.2769750202917223]
- **acc_noise_max_std**: 0.4261
- **deadband_ratio**: 5.326
- **above_deadband_pct**: 80.05
![layer3_imu_raw 3s-](out\layer3_imu_raw\3s-.png)

### [FAIL] layer3_imu_raw               4s
- **imu_count**: 3637
- **duration_s**: 36.37
- **quat_drift_deg**: 36.78
- **acc_mean_xyz**: [0.004063623865823475, 0.0056081660709375765, 0.017225900467418176]
- **acc_std_xyz**: [0.2586871749821736, 0.3666955431421494, 0.7776425950856551]
- **acc_noise_max_std**: 0.7776
- **deadband_ratio**: 9.721
- **above_deadband_pct**: 82.29
![layer3_imu_raw 4s](out\layer3_imu_raw\4s.png)

### [FAIL] layer3_imu_raw               4s-
- **imu_count**: 3564
- **duration_s**: 35.65
- **quat_drift_deg**: 29.55
- **acc_mean_xyz**: [0.009102132435465756, -0.003362289562289575, 0.013541329966329983]
- **acc_std_xyz**: [0.3033688460932739, 0.5452758922024742, 0.5237777803978728]
- **acc_noise_max_std**: 0.5453
- **deadband_ratio**: 6.816
- **above_deadband_pct**: 84.18
![layer3_imu_raw 4s-](out\layer3_imu_raw\4s-.png)

### [PASS] layer3_imu_raw               NorthS
- **imu_count**: 2369
- **duration_s**: 23.7
- **quat_drift_deg**: 2.895
- **acc_mean_xyz**: [-0.00095470662726889, -0.003967370198395943, 0.0018678345293372766]
- **acc_std_xyz**: [0.020349595793890087, 0.15263726959398788, 0.08947236830059108]
- **acc_noise_max_std**: 0.1526
- **deadband_ratio**: 1.908
- **above_deadband_pct**: 77.88
![layer3_imu_raw NorthS](out\layer3_imu_raw\NorthS.png)

### [PASS] layer3_imu_raw               SouthS
- **imu_count**: 2273
- **duration_s**: 22.76
- **quat_drift_deg**: 5.869
- **acc_mean_xyz**: [0.00031319841619005686, -0.00017496700395952706, -0.001976462824461063]
- **acc_std_xyz**: [0.021312785552375667, 0.11948819439192314, 0.08771971329047372]
- **acc_noise_max_std**: 0.1195
- **deadband_ratio**: 1.494
- **above_deadband_pct**: 71.62
![layer3_imu_raw SouthS](out\layer3_imu_raw\SouthS.png)

### [PASS] layer3_imu_raw               EastS
- **imu_count**: 2011
- **duration_s**: 20.13
- **quat_drift_deg**: 2.939
- **acc_mean_xyz**: [1.7553455992044047e-05, 0.0029267528592739895, 0.0034980606663351497]
- **acc_std_xyz**: [0.02545582273606698, 0.10984759873630998, 0.11701697531725959]
- **acc_noise_max_std**: 0.117
- **deadband_ratio**: 1.463
- **above_deadband_pct**: 72.9
![layer3_imu_raw EastS](out\layer3_imu_raw\EastS.png)

### [PASS] layer3_imu_raw               WestS
- **imu_count**: 1819
- **duration_s**: 18.2
- **quat_drift_deg**: 3.586
- **acc_mean_xyz**: [0.00094134139637163, 0.0013139087410665212, -0.001342935678944473]
- **acc_std_xyz**: [0.023357363683058165, 0.08355957695952115, 0.14049926494699236]
- **acc_noise_max_std**: 0.1405
- **deadband_ratio**: 1.756
- **above_deadband_pct**: 74.22
![layer3_imu_raw WestS](out\layer3_imu_raw\WestS.png)

### [FAIL] layer4_imu_integration       NorthS
- **n_imu**: 2369
- **samples_until_lock**: 41
- **locked_vec**: [0.9999944989245614, 0.0033169444697672182]
- **locked_heading_deg**: 0.19
- **post_lock_drift_deg**: 2.623
- **tag_z_error_mean_m**: 4.17e-06
- **tag_z_error_max_m**: 2.48e-05
- **motion_angle_deg**: -74.21
- **heading_vs_motion_deg**: 74.4
- **dr_drift_mean_m**: 0.02969
- **dr_drift_max_m**: 0.09014
![layer4_imu_integration NorthS](out\layer4_imu_integration\NorthS.png)

### [FAIL] layer4_imu_integration       SouthS
- **n_imu**: 2273
- **samples_until_lock**: 34
- **locked_vec**: [-0.9998295628371644, 0.018461995472991505]
- **locked_heading_deg**: 178.9
- **post_lock_drift_deg**: 1.403
- **tag_z_error_mean_m**: 0.0002842
- **tag_z_error_max_m**: 0.0005485
- **motion_angle_deg**: -23.49
- **heading_vs_motion_deg**: 22.43
- **dr_drift_mean_m**: 0.01987
- **dr_drift_max_m**: 0.07522
![layer4_imu_integration SouthS](out\layer4_imu_integration\SouthS.png)

### [FAIL] layer4_imu_integration       EastS
- **n_imu**: 2011
- **samples_until_lock**: 38
- **locked_vec**: [0.9443688366534422, -0.32888827944732896]
- **locked_heading_deg**: -19.2
- **post_lock_drift_deg**: 0
- **tag_z_error_mean_m**: 0.0001714
- **tag_z_error_max_m**: 0.0002685
- **motion_angle_deg**: -134.2
- **heading_vs_motion_deg**: 64.97
- **dr_drift_mean_m**: 0.05225
- **dr_drift_max_m**: 0.1515
![layer4_imu_integration EastS](out\layer4_imu_integration\EastS.png)

### [FAIL] layer4_imu_integration       WestS
- **n_imu**: 1819
- **samples_until_lock**: 32
- **locked_vec**: [-0.9793625799397001, -0.2021111996249948]
- **locked_heading_deg**: -168.3
- **post_lock_drift_deg**: 0.1454
- **tag_z_error_mean_m**: 0.0001599
- **tag_z_error_max_m**: 0.0003892
- **motion_angle_deg**: 82.81
- **heading_vs_motion_deg**: 71.15
- **dr_drift_mean_m**: 0.02634
- **dr_drift_max_m**: 0.1197
![layer4_imu_integration WestS](out\layer4_imu_integration\WestS.png)

### [FAIL] layer4_imu_integration       hline
- **n_imu**: 1557
- **samples_until_lock**: 30
- **locked_vec**: [0.9835211653952874, 0.18079302314938914]
- **locked_heading_deg**: 10.42
- **post_lock_drift_deg**: 10.47
- **tag_z_error_mean_m**: 0.0002074
- **tag_z_error_max_m**: 0.001274
- **motion_angle_deg**: 2.12
- **heading_vs_motion_deg**: 8.296
- **dr_drift_mean_m**: 0.1742
- **dr_drift_max_m**: 0.9131
![layer4_imu_integration hline](out\layer4_imu_integration\hline.png)

### [FAIL] layer4_imu_integration       vline
- **n_imu**: 1556
- **samples_until_lock**: 27
- **locked_vec**: [0.991337449784855, 0.1313394862714948]
- **locked_heading_deg**: 7.547
- **post_lock_drift_deg**: 12.14
- **tag_z_error_mean_m**: 0.002147
- **tag_z_error_max_m**: 0.004607
- **motion_angle_deg**: -89.18
- **heading_vs_motion_deg**: 83.27
- **dr_drift_mean_m**: 0.1083
- **dr_drift_max_m**: 0.5844
![layer4_imu_integration vline](out\layer4_imu_integration\vline.png)

### [PASS] layer5_force_contact         HELLO
- **imu_count**: 3021
- **raw_min**: 0
- **raw_max**: 234
- **raw_median**: 183
- **up_transitions**: 0
- **down_transitions**: 0
- **contact_episodes**: 0
- **mean_episode_samples**: 0
- **min_episode_samples**: 0
- **glitch_count**: 0
- **contact_duty_pct**: 0
![layer5_force_contact HELLO](out\layer5_force_contact\HELLO.png)

### [PASS] layer5_force_contact         ABC
- **imu_count**: 2561
- **raw_min**: 17
- **raw_max**: 244
- **raw_median**: 183
- **up_transitions**: 0
- **down_transitions**: 0
- **contact_episodes**: 0
- **mean_episode_samples**: 0
- **min_episode_samples**: 0
- **glitch_count**: 0
- **contact_duty_pct**: 0
![layer5_force_contact ABC](out\layer5_force_contact\ABC.png)

### [PASS] layer5_force_contact         CIRCLE
- **imu_count**: 1556
- **raw_min**: 15
- **raw_max**: 247
- **raw_median**: 191
- **up_transitions**: 0
- **down_transitions**: 0
- **contact_episodes**: 0
- **mean_episode_samples**: 0
- **min_episode_samples**: 0
- **glitch_count**: 0
- **contact_duty_pct**: 0
![layer5_force_contact CIRCLE](out\layer5_force_contact\CIRCLE.png)

### [PASS] layer5_force_contact         hline
- **imu_count**: 1557
- **raw_min**: 19
- **raw_max**: 244
- **raw_median**: 182
- **up_transitions**: 0
- **down_transitions**: 0
- **contact_episodes**: 0
- **mean_episode_samples**: 0
- **min_episode_samples**: 0
- **glitch_count**: 0
- **contact_duty_pct**: 0
![layer5_force_contact hline](out\layer5_force_contact\hline.png)

### [PASS] layer5_force_contact         0s
- **imu_count**: 2106
- **raw_min**: 21
- **raw_max**: 256
- **raw_median**: 197
- **up_transitions**: 0
- **down_transitions**: 0
- **contact_episodes**: 0
- **mean_episode_samples**: 0
- **min_episode_samples**: 0
- **glitch_count**: 0
- **contact_duty_pct**: 0
![layer5_force_contact 0s](out\layer5_force_contact\0s.png)

### [PASS] layer6_ekf_end_to_end        0s
- **uwb_updates**: 210
- **ekf_samples**: 206
- **cold_start_s**: 0.5
- **gate_reject_pct**: 0
- **accepted_total**: 824
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.01293
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.1
- **stationary_speed_mean**: 0.004251
- **bias_final_m_s2**: 0.01742
- **bias_delta_last_2s**: 0.000205
![layer6_ekf_end_to_end 0s](out\layer6_ekf_end_to_end\0s.png)

### [PASS] layer6_ekf_end_to_end        1s
- **uwb_updates**: 228
- **ekf_samples**: 224
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0
- **accepted_total**: 846
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0224
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.1
- **stationary_speed_mean**: 0.003975
- **bias_final_m_s2**: 0.004992
- **bias_delta_last_2s**: 0.0002249
![layer6_ekf_end_to_end 1s](out\layer6_ekf_end_to_end\1s.png)

### [FAIL] layer6_ekf_end_to_end        2s
- **uwb_updates**: 282
- **ekf_samples**: 278
- **cold_start_s**: 0.5
- **gate_reject_pct**: 0
- **accepted_total**: 1112
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.08791
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.1
- **stationary_speed_mean**: 0.003707
- **bias_final_m_s2**: 0.02502
- **bias_delta_last_2s**: 0.0001103
![layer6_ekf_end_to_end 2s](out\layer6_ekf_end_to_end\2s.png)

### [FAIL] layer6_ekf_end_to_end        3s
- **uwb_updates**: 318
- **ekf_samples**: 314
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0.1592
- **accepted_total**: 1254
- **rejected_total**: 2
- **rms_ekf_vs_irls_m**: 0.1469
- **latency_proxy_samples**: 28
- **latency_proxy_s**: 2.8
- **stationary_speed_mean**: 0.008208
- **bias_final_m_s2**: 0.005214
- **bias_delta_last_2s**: 0.0007162
![layer6_ekf_end_to_end 3s](out\layer6_ekf_end_to_end\3s.png)

### [FAIL] layer6_ekf_end_to_end        4s
- **uwb_updates**: 363
- **ekf_samples**: 359
- **cold_start_s**: 0.5
- **gate_reject_pct**: 0.97
- **accepted_total**: 1123
- **rejected_total**: 11
- **rms_ekf_vs_irls_m**: 0.1784
- **latency_proxy_samples**: 5
- **latency_proxy_s**: 0.5
- **stationary_speed_mean**: 0.01268
- **bias_final_m_s2**: 0.02706
- **bias_delta_last_2s**: 0.00371
![layer6_ekf_end_to_end 4s](out\layer6_ekf_end_to_end\4s.png)

### [FAIL] layer6_ekf_end_to_end        NorthS
- **uwb_updates**: 237
- **ekf_samples**: 233
- **cold_start_s**: 0.5
- **gate_reject_pct**: 0
- **accepted_total**: 932
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.01495
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.1
- **stationary_speed_mean**: 0.004481
- **bias_final_m_s2**: 0.003724
- **bias_delta_last_2s**: 0.0005833
![layer6_ekf_end_to_end NorthS](out\layer6_ekf_end_to_end\NorthS.png)

### [PASS] layer6_ekf_end_to_end        SouthS
- **uwb_updates**: 227
- **ekf_samples**: 223
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0
- **accepted_total**: 892
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.01027
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.1
- **stationary_speed_mean**: 0.003811
- **bias_final_m_s2**: 0.005623
- **bias_delta_last_2s**: 0.0001004
![layer6_ekf_end_to_end SouthS](out\layer6_ekf_end_to_end\SouthS.png)

### [FAIL] layer6_ekf_end_to_end        EastS
- **uwb_updates**: 201
- **ekf_samples**: 197
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0
- **accepted_total**: 788
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04439
- **latency_proxy_samples**: 2
- **latency_proxy_s**: 0.2
- **stationary_speed_mean**: 0.003833
- **bias_final_m_s2**: 0.008291
- **bias_delta_last_2s**: 0.0001381
![layer6_ekf_end_to_end EastS](out\layer6_ekf_end_to_end\EastS.png)

### [PASS] layer6_ekf_end_to_end        WestS
- **uwb_updates**: 181
- **ekf_samples**: 177
- **cold_start_s**: 0.5
- **gate_reject_pct**: 0
- **accepted_total**: 708
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.01263
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.1
- **stationary_speed_mean**: 0.004183
- **bias_final_m_s2**: 0.005989
- **bias_delta_last_2s**: 0.0002357
![layer6_ekf_end_to_end WestS](out\layer6_ekf_end_to_end\WestS.png)

### [FAIL] layer6_ekf_end_to_end        hline
- **uwb_updates**: 155
- **ekf_samples**: 151
- **cold_start_s**: 0.5
- **gate_reject_pct**: 0
- **accepted_total**: 604
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.09347
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.01323
- **bias_final_m_s2**: 0.1001
- **bias_delta_last_2s**: 0.0002956
![layer6_ekf_end_to_end hline](out\layer6_ekf_end_to_end\hline.png)

### [FAIL] layer6_ekf_end_to_end        vline
- **uwb_updates**: 152
- **ekf_samples**: 148
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0
- **accepted_total**: 592
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.06837
- **latency_proxy_samples**: 2
- **latency_proxy_s**: 0.2
- **stationary_speed_mean**: 0.01431
- **bias_final_m_s2**: 0.0873
- **bias_delta_last_2s**: 0.003342
![layer6_ekf_end_to_end vline](out\layer6_ekf_end_to_end\vline.png)

### [FAIL] layer6_ekf_end_to_end        dline_A0_A2
- **uwb_updates**: 182
- **ekf_samples**: 178
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0.3096
- **accepted_total**: 644
- **rejected_total**: 2
- **rms_ekf_vs_irls_m**: 0.1491
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.01748
- **bias_final_m_s2**: 0.1089
- **bias_delta_last_2s**: 0.005378
![layer6_ekf_end_to_end dline_A0_A2](out\layer6_ekf_end_to_end\dline_A0_A2.png)

### [FAIL] layer6_ekf_end_to_end        dline_A3_A1
- **uwb_updates**: 154
- **ekf_samples**: 150
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0.3738
- **accepted_total**: 533
- **rejected_total**: 2
- **rms_ekf_vs_irls_m**: 0.1439
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.01569
- **bias_final_m_s2**: 0.08426
- **bias_delta_last_2s**: 0.0059
![layer6_ekf_end_to_end dline_A3_A1](out\layer6_ekf_end_to_end\dline_A3_A1.png)

### [FAIL] layer6_ekf_end_to_end        CIRCLE
- **uwb_updates**: 156
- **ekf_samples**: 152
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0.3289
- **accepted_total**: 606
- **rejected_total**: 2
- **rms_ekf_vs_irls_m**: 0.0879
- **latency_proxy_samples**: 5
- **latency_proxy_s**: 0.5
- **stationary_speed_mean**: 0.0168
- **bias_final_m_s2**: 0.04173
- **bias_delta_last_2s**: 0.002555
![layer6_ekf_end_to_end CIRCLE](out\layer6_ekf_end_to_end\CIRCLE.png)

### [FAIL] layer6_ekf_end_to_end        SQUARE
- **uwb_updates**: 207
- **ekf_samples**: 203
- **cold_start_s**: 0.5
- **gate_reject_pct**: 0.2463
- **accepted_total**: 810
- **rejected_total**: 2
- **rms_ekf_vs_irls_m**: 0.08979
- **latency_proxy_samples**: 6
- **latency_proxy_s**: 0.6
- **stationary_speed_mean**: 0.01695
- **bias_final_m_s2**: 0.01092
- **bias_delta_last_2s**: 0.003485
![layer6_ekf_end_to_end SQUARE](out\layer6_ekf_end_to_end\SQUARE.png)

### [FAIL] layer6_ekf_end_to_end        TRIANGLE
- **uwb_updates**: 182
- **ekf_samples**: 178
- **cold_start_s**: 0.4
- **gate_reject_pct**: 0
- **accepted_total**: 712
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.08273
- **latency_proxy_samples**: 6
- **latency_proxy_s**: 0.6
- **stationary_speed_mean**: 0.01533
- **bias_final_m_s2**: 0.0325
- **bias_delta_last_2s**: 0.0007603
![layer6_ekf_end_to_end TRIANGLE](out\layer6_ekf_end_to_end\TRIANGLE.png)
