# Async-stream verification report

Datasets root: `C:\Users\Christian Villanueva\Documents\College Files\Thesis\esp32_mock_webserver\kuru_method\asynchronous_stream\datasets_str_50hz`


## Per-dataset summary

| Dataset | First fail | Layers run |
|---|---|---|
| ABC_b1 | layer0_stream_health | 3 |
| ABC_b1+contact | layer6_ekf_end_to_end | 1 |
| ABC_b2 | -- | 3 |
| ABC_b2+contact | -- | 1 |
| HELLO_b1 | -- | 3 |
| HELLO_b1+contact | -- | 1 |
| HELLO_b2 | -- | 3 |
| HELLO_b2+contact | -- | 1 |
| abc_s1 | layer0_stream_health | 3 |
| abc_s1+contact | -- | 1 |
| abc_s2 | layer0_stream_health | 3 |
| abc_s2+contact | -- | 1 |
| ct_circle1 | layer0_stream_health | 5 |
| ct_circle1+contact | -- | 1 |
| ct_circle2 | layer0_stream_health | 5 |
| ct_circle2+contact | -- | 1 |
| ct_circle3 | layer4_imu_integration | 5 |
| ct_circle3+contact | -- | 1 |
| ct_diagonal1 | layer2_uwb_position | 5 |
| ct_diagonal1+contact | -- | 1 |
| ct_diagonal2 | layer0_stream_health | 5 |
| ct_diagonal2+contact | -- | 1 |
| ct_diagonal3 | layer0_stream_health | 5 |
| ct_diagonal3+contact | -- | 1 |
| ct_hline1 | layer0_stream_health | 5 |
| ct_hline1+contact | -- | 1 |
| ct_hline2 | layer0_stream_health | 5 |
| ct_hline2+contact | -- | 1 |
| ct_hline3 | layer0_stream_health | 5 |
| ct_hline3+contact | -- | 1 |
| ct_square1 | layer0_stream_health | 5 |
| ct_square1+contact | -- | 1 |
| ct_square2 | layer0_stream_health | 5 |
| ct_square2+contact | -- | 1 |
| ct_square3 | layer0_stream_health | 5 |
| ct_square3+contact | -- | 1 |
| ct_triangle1 | layer4_imu_integration | 5 |
| ct_triangle1+contact | -- | 1 |
| ct_triangle2 | layer0_stream_health | 5 |
| ct_triangle2+contact | -- | 1 |
| ct_triangle3 | layer4_imu_integration | 5 |
| ct_triangle3+contact | -- | 1 |
| ct_vline1 | layer0_stream_health | 5 |
| ct_vline1+contact | -- | 1 |
| ct_vline2 | layer0_stream_health | 5 |
| ct_vline2+contact | -- | 1 |
| ct_vline3 | layer0_stream_health | 5 |
| ct_vline3+contact | layer6_ekf_end_to_end | 1 |
| hello_s1 | -- | 3 |
| hello_s1+contact | -- | 1 |
| hello_s2 | layer0_stream_health | 3 |
| hello_s2+contact | -- | 1 |
| imu_a | layer0_stream_health | 2 |
| imu_circle | layer0_stream_health | 2 |
| imu_square | layer0_stream_health | 2 |
| imu_triangle | layer0_stream_health | 2 |
| middle- | layer0_stream_health | 6 |
| middleCCW_rot- | layer0_stream_health | 5 |
| middleCW_rot- | layer2_uwb_position | 5 |
| pt_a1 | layer2_uwb_position | 4 |
| pt_a1+contact | -- | 1 |
| pt_a2 | layer2_uwb_position | 4 |
| pt_a2+contact | layer6_ekf_end_to_end | 1 |
| pt_circle1 | layer4_imu_integration | 5 |
| pt_circle1+contact | -- | 1 |
| pt_circle2 | layer0_stream_health | 5 |
| pt_circle2+contact | -- | 1 |
| pt_square1 | layer4_imu_integration | 5 |
| pt_square1+contact | -- | 1 |
| pt_square2 | layer4_imu_integration | 5 |
| pt_square2+contact | -- | 1 |
| pt_triangle1 | layer4_imu_integration | 5 |
| pt_triangle1+contact | -- | 1 |
| pt_triangle2 | layer4_imu_integration | 5 |
| pt_triangle2+contact | -- | 1 |
| rt_circle | layer2_uwb_position | 5 |
| rt_square | layer2_uwb_position | 5 |
| rt_triangle | layer2_uwb_position | 5 |

## Detailed results

### [FAIL] layer0_stream_health         ABC_b1
- **imu_count**: 3851
- **uwb_count**: 1102
- **duration_s**: 22.98
- **imu_rate_hz**: 217.6
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 4.418
- **uwb_loss_pct**: 4.091
- **imu_dt_spike_ms**: 64.12
- **uwb_dt_spike_ms**: 180
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ABC_b1](out\layer0_stream_health\ABC_b1.png)

### [PASS] layer0_stream_health         ABC_b2
- **imu_count**: 3885
- **uwb_count**: 1115
- **duration_s**: 22.62
- **imu_rate_hz**: 216.2
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.195
- **uwb_loss_pct**: 1.415
- **imu_dt_spike_ms**: 40.92
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ABC_b2](out\layer0_stream_health\ABC_b2.png)

### [FAIL] layer0_stream_health         abc_s1
- **imu_count**: 3203
- **uwb_count**: 910
- **duration_s**: 19.02
- **imu_rate_hz**: 219.3
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 3.349
- **uwb_loss_pct**: 4.311
- **imu_dt_spike_ms**: 76.49
- **uwb_dt_spike_ms**: 160
- **imu_rate_at_200hz_target**: True
![layer0_stream_health abc_s1](out\layer0_stream_health\abc_s1.png)

### [FAIL] layer0_stream_health         abc_s2
- **imu_count**: 2825
- **uwb_count**: 789
- **duration_s**: 17.97
- **imu_rate_hz**: 216.3
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 11.72
- **uwb_loss_pct**: 12.24
- **imu_dt_spike_ms**: 260.2
- **uwb_dt_spike_ms**: 500
- **imu_rate_at_200hz_target**: True
![layer0_stream_health abc_s2](out\layer0_stream_health\abc_s2.png)

### [FAIL] layer0_stream_health         ct_circle1
- **imu_count**: 1800
- **uwb_count**: 516
- **duration_s**: 11.41
- **imu_rate_hz**: 213.2
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 9.228
- **uwb_loss_pct**: 9.948
- **imu_dt_spike_ms**: 77.98
- **uwb_dt_spike_ms**: 300
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_circle1](out\layer0_stream_health\ct_circle1.png)

### [FAIL] layer0_stream_health         ct_circle2
- **imu_count**: 2131
- **uwb_count**: 609
- **duration_s**: 12.36
- **imu_rate_hz**: 218
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.023
- **uwb_loss_pct**: 1.456
- **imu_dt_spike_ms**: 31.26
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_circle2](out\layer0_stream_health\ct_circle2.png)

### [PASS] layer0_stream_health         ct_circle3
- **imu_count**: 2039
- **uwb_count**: 579
- **duration_s**: 11.75
- **imu_rate_hz**: 219.9
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.64
- **uwb_loss_pct**: 1.531
- **imu_dt_spike_ms**: 30.9
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_circle3](out\layer0_stream_health\ct_circle3.png)

### [PASS] layer0_stream_health         ct_diagonal1
- **imu_count**: 1833
- **uwb_count**: 514
- **duration_s**: 10.42
- **imu_rate_hz**: 221.1
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.026
- **uwb_loss_pct**: 1.344
- **imu_dt_spike_ms**: 30.31
- **uwb_dt_spike_ms**: 120
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_diagonal1](out\layer0_stream_health\ct_diagonal1.png)

### [FAIL] layer0_stream_health         ct_diagonal2
- **imu_count**: 1564
- **uwb_count**: 436
- **duration_s**: 8.934
- **imu_rate_hz**: 221.5
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.797
- **uwb_loss_pct**: 2.461
- **imu_dt_spike_ms**: 32.17
- **uwb_dt_spike_ms**: 120
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_diagonal2](out\layer0_stream_health\ct_diagonal2.png)

### [FAIL] layer0_stream_health         ct_diagonal3
- **imu_count**: 1731
- **uwb_count**: 497
- **duration_s**: 10.13
- **imu_rate_hz**: 212.9
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.037
- **uwb_loss_pct**: 1.779
- **imu_dt_spike_ms**: 31.24
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_diagonal3](out\layer0_stream_health\ct_diagonal3.png)

### [FAIL] layer0_stream_health         ct_hline1
- **imu_count**: 1538
- **uwb_count**: 429
- **duration_s**: 8.733
- **imu_rate_hz**: 227.2
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.41
- **uwb_loss_pct**: 2.721
- **imu_dt_spike_ms**: 28.91
- **uwb_dt_spike_ms**: 140
- **imu_rate_at_200hz_target**: False
![layer0_stream_health ct_hline1](out\layer0_stream_health\ct_hline1.png)

### [FAIL] layer0_stream_health         ct_hline2
- **imu_count**: 1565
- **uwb_count**: 435
- **duration_s**: 9.05
- **imu_rate_hz**: 219.6
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 3.514
- **uwb_loss_pct**: 3.761
- **imu_dt_spike_ms**: 67.71
- **uwb_dt_spike_ms**: 160
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_hline2](out\layer0_stream_health\ct_hline2.png)

### [FAIL] layer0_stream_health         ct_hline3
- **imu_count**: 1526
- **uwb_count**: 438
- **duration_s**: 10.42
- **imu_rate_hz**: 212.9
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 17.29
- **uwb_loss_pct**: 15.93
- **imu_dt_spike_ms**: 395.2
- **uwb_dt_spike_ms**: 380
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_hline3](out\layer0_stream_health\ct_hline3.png)

### [FAIL] layer0_stream_health         ct_square1
- **imu_count**: 2187
- **uwb_count**: 635
- **duration_s**: 12.92
- **imu_rate_hz**: 216.7
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.13
- **uwb_loss_pct**: 2.157
- **imu_dt_spike_ms**: 32.02
- **uwb_dt_spike_ms**: 119
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_square1](out\layer0_stream_health\ct_square1.png)

### [FAIL] layer0_stream_health         ct_square2
- **imu_count**: 2323
- **uwb_count**: 677
- **duration_s**: 13.81
- **imu_rate_hz**: 218.2
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.272
- **uwb_loss_pct**: 2.026
- **imu_dt_spike_ms**: 46.49
- **uwb_dt_spike_ms**: 81
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_square2](out\layer0_stream_health\ct_square2.png)

### [FAIL] layer0_stream_health         ct_square3
- **imu_count**: 2334
- **uwb_count**: 666
- **duration_s**: 13.79
- **imu_rate_hz**: 218.7
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 3.594
- **uwb_loss_pct**: 3.478
- **imu_dt_spike_ms**: 61.27
- **uwb_dt_spike_ms**: 180
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_square3](out\layer0_stream_health\ct_square3.png)

### [PASS] layer0_stream_health         ct_triangle1
- **imu_count**: 1943
- **uwb_count**: 557
- **duration_s**: 11.33
- **imu_rate_hz**: 218.1
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.32
- **uwb_loss_pct**: 1.764
- **imu_dt_spike_ms**: 32.46
- **uwb_dt_spike_ms**: 200
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_triangle1](out\layer0_stream_health\ct_triangle1.png)

### [FAIL] layer0_stream_health         ct_triangle2
- **imu_count**: 2136
- **uwb_count**: 598
- **duration_s**: 12.19
- **imu_rate_hz**: 224.3
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.153
- **uwb_loss_pct**: 1.967
- **imu_dt_spike_ms**: 57.05
- **uwb_dt_spike_ms**: 120
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_triangle2](out\layer0_stream_health\ct_triangle2.png)

### [PASS] layer0_stream_health         ct_triangle3
- **imu_count**: 1833
- **uwb_count**: 519
- **duration_s**: 10.56
- **imu_rate_hz**: 221
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 0.9724
- **uwb_loss_pct**: 1.705
- **imu_dt_spike_ms**: 29.65
- **uwb_dt_spike_ms**: 160
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_triangle3](out\layer0_stream_health\ct_triangle3.png)

### [FAIL] layer0_stream_health         ct_vline1
- **imu_count**: 1251
- **uwb_count**: 366
- **duration_s**: 7.617
- **imu_rate_hz**: 215.2
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 3.472
- **uwb_loss_pct**: 3.937
- **imu_dt_spike_ms**: 39.09
- **uwb_dt_spike_ms**: 120
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_vline1](out\layer0_stream_health\ct_vline1.png)

### [FAIL] layer0_stream_health         ct_vline2
- **imu_count**: 1550
- **uwb_count**: 450
- **duration_s**: 9.386
- **imu_rate_hz**: 218.3
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 3.427
- **uwb_loss_pct**: 4.255
- **imu_dt_spike_ms**: 60.85
- **uwb_dt_spike_ms**: 220
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_vline2](out\layer0_stream_health\ct_vline2.png)

### [FAIL] layer0_stream_health         ct_vline3
- **imu_count**: 1650
- **uwb_count**: 467
- **duration_s**: 9.73
- **imu_rate_hz**: 222.3
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 4.07
- **uwb_loss_pct**: 4.107
- **imu_dt_spike_ms**: 50.34
- **uwb_dt_spike_ms**: 120
- **imu_rate_at_200hz_target**: True
![layer0_stream_health ct_vline3](out\layer0_stream_health\ct_vline3.png)

### [PASS] layer0_stream_health         HELLO_b1
- **imu_count**: 5239
- **uwb_count**: 1492
- **duration_s**: 30.24
- **imu_rate_hz**: 218.7
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.132
- **uwb_loss_pct**: 1.323
- **imu_dt_spike_ms**: 44.11
- **uwb_dt_spike_ms**: 200
- **imu_rate_at_200hz_target**: True
![layer0_stream_health HELLO_b1](out\layer0_stream_health\HELLO_b1.png)

### [PASS] layer0_stream_health         HELLO_b2
- **imu_count**: 5150
- **uwb_count**: 1463
- **duration_s**: 29.38
- **imu_rate_hz**: 219.4
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 0.4831
- **uwb_loss_pct**: 0.4084
- **imu_dt_spike_ms**: 49.27
- **uwb_dt_spike_ms**: 80
- **imu_rate_at_200hz_target**: True
![layer0_stream_health HELLO_b2](out\layer0_stream_health\HELLO_b2.png)

### [PASS] layer0_stream_health         hello_s1
- **imu_count**: 3075
- **uwb_count**: 885
- **duration_s**: 17.88
- **imu_rate_hz**: 216.8
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.157
- **uwb_loss_pct**: 1.117
- **imu_dt_spike_ms**: 30.31
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health hello_s1](out\layer0_stream_health\hello_s1.png)

### [FAIL] layer0_stream_health         hello_s2
- **imu_count**: 2913
- **uwb_count**: 819
- **duration_s**: 17.06
- **imu_rate_hz**: 218.8
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.868
- **uwb_loss_pct**: 3.986
- **imu_dt_spike_ms**: 50.3
- **uwb_dt_spike_ms**: 160
- **imu_rate_at_200hz_target**: True
![layer0_stream_health hello_s2](out\layer0_stream_health\hello_s2.png)

### [FAIL] layer0_stream_health         imu_a
- **imu_count**: 1539
- **uwb_count**: 461
- **duration_s**: 9.273
- **imu_rate_hz**: 223.3
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 5.814
- **uwb_loss_pct**: 7.43
- **imu_dt_spike_ms**: 90.37
- **uwb_dt_spike_ms**: 160
- **imu_rate_at_200hz_target**: True
![layer0_stream_health imu_a](out\layer0_stream_health\imu_a.png)

### [FAIL] layer0_stream_health         imu_circle
- **imu_count**: 1355
- **uwb_count**: 383
- **duration_s**: 7.822
- **imu_rate_hz**: 219.1
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.518
- **uwb_loss_pct**: 2.296
- **imu_dt_spike_ms**: 30.71
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health imu_circle](out\layer0_stream_health\imu_circle.png)

### [FAIL] layer0_stream_health         imu_square
- **imu_count**: 1440
- **uwb_count**: 418
- **duration_s**: 8.571
- **imu_rate_hz**: 212
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.373
- **uwb_loss_pct**: 2.336
- **imu_dt_spike_ms**: 43.04
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health imu_square](out\layer0_stream_health\imu_square.png)

### [FAIL] layer0_stream_health         imu_triangle
- **imu_count**: 1559
- **uwb_count**: 441
- **duration_s**: 9.141
- **imu_rate_hz**: 218.2
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.745
- **uwb_loss_pct**: 3.501
- **imu_dt_spike_ms**: 35.07
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health imu_triangle](out\layer0_stream_health\imu_triangle.png)

### [FAIL] layer0_stream_health         middle-
- **imu_count**: 3007
- **uwb_count**: 833
- **duration_s**: 18.27
- **imu_rate_hz**: 219.5
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 8.043
- **uwb_loss_pct**: 8.762
- **imu_dt_spike_ms**: 70.64
- **uwb_dt_spike_ms**: 240
- **imu_rate_at_200hz_target**: True
![layer0_stream_health middle-](out\layer0_stream_health\middle-.png)

### [FAIL] layer0_stream_health         middleCCW_rot-
- **imu_count**: 4632
- **uwb_count**: 1315
- **duration_s**: 26.87
- **imu_rate_hz**: 219.5
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.844
- **uwb_loss_pct**: 2.158
- **imu_dt_spike_ms**: 39.45
- **uwb_dt_spike_ms**: 200
- **imu_rate_at_200hz_target**: True
![layer0_stream_health middleCCW_rot-](out\layer0_stream_health\middleCCW_rot-.png)

### [PASS] layer0_stream_health         middleCW_rot-
- **imu_count**: 5014
- **uwb_count**: 1434
- **duration_s**: 28.94
- **imu_rate_hz**: 218.9
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 0.7129
- **uwb_loss_pct**: 0.8984
- **imu_dt_spike_ms**: 28.19
- **uwb_dt_spike_ms**: 200
- **imu_rate_at_200hz_target**: True
![layer0_stream_health middleCW_rot-](out\layer0_stream_health\middleCW_rot-.png)

### [PASS] layer0_stream_health         pt_a1
- **imu_count**: 1822
- **uwb_count**: 523
- **duration_s**: 10.63
- **imu_rate_hz**: 213.4
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.938
- **uwb_loss_pct**: 1.692
- **imu_dt_spike_ms**: 30.62
- **uwb_dt_spike_ms**: 120
- **imu_rate_at_200hz_target**: True
![layer0_stream_health pt_a1](out\layer0_stream_health\pt_a1.png)

### [PASS] layer0_stream_health         pt_a2
- **imu_count**: 1739
- **uwb_count**: 488
- **duration_s**: 9.907
- **imu_rate_hz**: 221.1
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.249
- **uwb_loss_pct**: 1.414
- **imu_dt_spike_ms**: 44.7
- **uwb_dt_spike_ms**: 80
- **imu_rate_at_200hz_target**: True
![layer0_stream_health pt_a2](out\layer0_stream_health\pt_a2.png)

### [PASS] layer0_stream_health         pt_circle1
- **imu_count**: 2221
- **uwb_count**: 621
- **duration_s**: 12.61
- **imu_rate_hz**: 222
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 0.9367
- **uwb_loss_pct**: 1.429
- **imu_dt_spike_ms**: 29.86
- **uwb_dt_spike_ms**: 180
- **imu_rate_at_200hz_target**: True
![layer0_stream_health pt_circle1](out\layer0_stream_health\pt_circle1.png)

### [FAIL] layer0_stream_health         pt_circle2
- **imu_count**: 2136
- **uwb_count**: 594
- **duration_s**: 12.29
- **imu_rate_hz**: 219.7
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 2.777
- **uwb_loss_pct**: 3.257
- **imu_dt_spike_ms**: 40.71
- **uwb_dt_spike_ms**: 160
- **imu_rate_at_200hz_target**: True
![layer0_stream_health pt_circle2](out\layer0_stream_health\pt_circle2.png)

### [PASS] layer0_stream_health         pt_square1
- **imu_count**: 2785
- **uwb_count**: 832
- **duration_s**: 16.12
- **imu_rate_hz**: 219.9
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.381
- **uwb_loss_pct**: 1.422
- **imu_dt_spike_ms**: 50.78
- **uwb_dt_spike_ms**: 120
- **imu_rate_at_200hz_target**: True
![layer0_stream_health pt_square1](out\layer0_stream_health\pt_square1.png)

### [PASS] layer0_stream_health         pt_square2
- **imu_count**: 2604
- **uwb_count**: 732
- **duration_s**: 14.84
- **imu_rate_hz**: 223.3
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.064
- **uwb_loss_pct**: 1.348
- **imu_dt_spike_ms**: 31.03
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health pt_square2](out\layer0_stream_health\pt_square2.png)

### [PASS] layer0_stream_health         pt_triangle1
- **imu_count**: 2443
- **uwb_count**: 673
- **duration_s**: 13.67
- **imu_rate_hz**: 223.5
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.492
- **uwb_loss_pct**: 1.464
- **imu_dt_spike_ms**: 31.5
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health pt_triangle1](out\layer0_stream_health\pt_triangle1.png)

### [PASS] layer0_stream_health         pt_triangle2
- **imu_count**: 2351
- **uwb_count**: 649
- **duration_s**: 13.17
- **imu_rate_hz**: 223.7
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.011
- **uwb_loss_pct**: 1.517
- **imu_dt_spike_ms**: 28.75
- **uwb_dt_spike_ms**: 160
- **imu_rate_at_200hz_target**: True
![layer0_stream_health pt_triangle2](out\layer0_stream_health\pt_triangle2.png)

### [PASS] layer0_stream_health         rt_circle
- **imu_count**: 3279
- **uwb_count**: 951
- **duration_s**: 18.61
- **imu_rate_hz**: 223.1
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 0.9964
- **uwb_loss_pct**: 1.349
- **imu_dt_spike_ms**: 31.4
- **uwb_dt_spike_ms**: 140
- **imu_rate_at_200hz_target**: True
![layer0_stream_health rt_circle](out\layer0_stream_health\rt_circle.png)

### [PASS] layer0_stream_health         rt_square
- **imu_count**: 3291
- **uwb_count**: 937
- **duration_s**: 18.92
- **imu_rate_hz**: 218.3
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 0.574
- **uwb_loss_pct**: 0.9514
- **imu_dt_spike_ms**: 25.67
- **uwb_dt_spike_ms**: 160
- **imu_rate_at_200hz_target**: True
![layer0_stream_health rt_square](out\layer0_stream_health\rt_square.png)

### [PASS] layer0_stream_health         rt_triangle
- **imu_count**: 3108
- **uwb_count**: 892
- **duration_s**: 18.04
- **imu_rate_hz**: 218.2
- **uwb_rate_hz**: 50
- **imu_loss_pct**: 1.208
- **uwb_loss_pct**: 1.218
- **imu_dt_spike_ms**: 36.01
- **uwb_dt_spike_ms**: 100
- **imu_rate_at_200hz_target**: True
![layer0_stream_health rt_triangle](out\layer0_stream_health\rt_triangle.png)

### [PASS] layer1_uwb_raw               middle-
- **uwb_count**: 833
- **raw_mean**: [0.9789675870348237, 0.9801800720288159, 1.120744297719088, 1.0287034813925506]
- **raw_std**: [0.021618545150349434, 0.027331783651892316, 0.02636748642235527, 0.03295746030204041]
- **cal_mean**: [0.825167587034807, 0.966780072028806, 0.9374442977190817, 0.9327034813925567]
- **cal_std**: [0.02161854515034939, 0.027331783651892348, 0.02636748642235527, 0.0329574603020404]
- **des_mean**: [0.825167587034807, 0.966780072028806, 0.9374442977190817, 0.9327034813925567]
- **des_std**: [0.02161854515034939, 0.027331783651892348, 0.02636748642235527, 0.0329574603020404]
- **kf_mean**: [0.9786910984234733, 0.9794738828108265, 1.1206049996584235, 1.028722332491582]
- **kf_std**: [0.01029088351437198, 0.01958064557510479, 0.014831925219106992, 0.014926511377396147]
- **var_reduction_pct**: [77.34039180135699, 48.67628742802164, 68.35846148255894, 79.48794570612895]
- **min_var_reduction_pct**: 48.68
- **expected**: [0.894776508408664, 0.894776508408664, 0.894776508408664, 0.894776508408664]
- **raw_bias**: [0.08419107862615971, 0.08540356362015189, 0.22596778931042405, 0.13392697298388667]
- **cal_bias**: [-0.06960892137385699, 0.07200356362014204, 0.0426677893104177, 0.03792697298389269]
- **des_bias**: [-0.06960892137385699, 0.07200356362014204, 0.0426677893104177, 0.03792697298389269]
- **kf_bias**: [0.08391459001480939, 0.08469737440216252, 0.22582849124975957, 0.133945824082918]
- **truth**: (0.625, 0.62)
- **max_cal_abs_bias**: 0.072
- **max_cal_std**: 0.03296
![layer1_uwb_raw middle-](out\layer1_uwb_raw\middle-.png)

### [FAIL] layer2_uwb_position          middle-
- **n_points**: 833
- **kind**: stationary
- **mean**: [0.570847595772719, 0.5884471626012296]
- **std**: [0.019739242791787966, 0.020309309014498036]
- **r95_m**: 0.04772
- **kf_r95_m**: 0.03071
- **truth**: (0.625, 0.62)
- **bias_m**: 0.06267
- **kf_irls_rms_m**: 0.06487
- **irls_rms_m**: 0.06878
![layer2_uwb_position middle-](out\layer2_uwb_position\middle-.png)

### [FAIL] layer2_uwb_position          pt_a1
- **n_points**: 523
- **kind**: stationary
- **mean**: [0.5890641023419013, 0.6150803206481589]
- **std**: [0.02733774069853988, 0.0353336803976739]
- **r95_m**: 0.08045
- **kf_r95_m**: 0.07996
> no ground truth -- passed-criterion uses R95 only
![layer2_uwb_position pt_a1](out\layer2_uwb_position\pt_a1.png)

### [FAIL] layer2_uwb_position          pt_a2
- **n_points**: 488
- **kind**: stationary
- **mean**: [0.6616473626210705, 0.6184266748326737]
- **std**: [0.03678876888016242, 0.04948813042424651]
- **r95_m**: 0.1103
- **kf_r95_m**: 0.1233
> no ground truth -- passed-criterion uses R95 only
![layer2_uwb_position pt_a2](out\layer2_uwb_position\pt_a2.png)

### [FAIL] layer2_uwb_position          middleCW_rot-
- **n_points**: 1434
- **kind**: rotation
- **mean**: [0.5967725924407463, 0.5735974178744175]
- **std**: [0.055947248143139394, 0.05937646346232985]
- **r95_m**: 0.1297
- **kf_r95_m**: 0.1245
- **truth**: (0.625, 0.62)
- **bias_m**: 0.05431
- **kf_irls_rms_m**: 0.09119
- **irls_rms_m**: 0.09801
![layer2_uwb_position middleCW_rot-](out\layer2_uwb_position\middleCW_rot-.png)

### [FAIL] layer2_uwb_position          rt_square
- **n_points**: 937
- **kind**: rotation
- **mean**: [0.5931673166350575, 0.6523995979838096]
- **std**: [0.1307943384251729, 0.09720233642106212]
- **r95_m**: 0.2451
- **kf_r95_m**: 0.267
> no ground truth -- passed-criterion uses R95 only
![layer2_uwb_position rt_square](out\layer2_uwb_position\rt_square.png)

### [FAIL] layer2_uwb_position          rt_circle
- **n_points**: 951
- **kind**: rotation
- **mean**: [0.5960212205180296, 0.6627103834160782]
- **std**: [0.1000318272908139, 0.1027646993551602]
- **r95_m**: 0.2251
- **kf_r95_m**: 0.2599
> no ground truth -- passed-criterion uses R95 only
![layer2_uwb_position rt_circle](out\layer2_uwb_position\rt_circle.png)

### [FAIL] layer2_uwb_position          rt_triangle
- **n_points**: 892
- **kind**: rotation
- **mean**: [0.61407062789589, 0.6426494265060257]
- **std**: [0.1024690805771237, 0.11441273412056063]
- **r95_m**: 0.2493
- **kf_r95_m**: 0.2674
> no ground truth -- passed-criterion uses R95 only
![layer2_uwb_position rt_triangle](out\layer2_uwb_position\rt_triangle.png)

### [FAIL] layer2_uwb_position          middleCCW_rot-
- **n_points**: 1315
- **kind**: rotation
- **mean**: [0.5892906212981712, 0.5514393680182704]
- **std**: [0.056854359085310655, 0.08854729276253677]
- **r95_m**: 0.2384
- **kf_r95_m**: 0.205
- **truth**: (0.625, 0.62)
- **bias_m**: 0.0773
- **kf_irls_rms_m**: 0.1299
- **irls_rms_m**: 0.1306
![layer2_uwb_position middleCCW_rot-](out\layer2_uwb_position\middleCCW_rot-.png)

### [FAIL] layer2_uwb_position          ct_diagonal1
- **n_points**: 514
- **kind**: line_d02
- **angle_deg**: -38.15
- **expected_deg**: 44.77
- **angle_err_deg**: 82.92
- **span_m**: 1.139
- **rms_perp_m**: 0.02321
![layer2_uwb_position ct_diagonal1](out\layer2_uwb_position\ct_diagonal1.png)

### [FAIL] layer2_uwb_position          ct_diagonal2
- **n_points**: 436
- **kind**: line_d02
- **angle_deg**: -40.22
- **expected_deg**: 44.77
- **angle_err_deg**: 84.99
- **span_m**: 1.089
- **rms_perp_m**: 0.03179
![layer2_uwb_position ct_diagonal2](out\layer2_uwb_position\ct_diagonal2.png)

### [FAIL] layer2_uwb_position          ct_diagonal3
- **n_points**: 497
- **kind**: line_d02
- **angle_deg**: -39.17
- **expected_deg**: 44.77
- **angle_err_deg**: 83.94
- **span_m**: 1.174
- **rms_perp_m**: 0.02644
![layer2_uwb_position ct_diagonal3](out\layer2_uwb_position\ct_diagonal3.png)

### [PASS] layer2_uwb_position          ct_hline1
- **n_points**: 429
- **kind**: line_h
- **angle_deg**: 0.2241
- **expected_deg**: 0
- **angle_err_deg**: 0.2241
- **span_m**: 0.9071
- **rms_perp_m**: 0.02037
![layer2_uwb_position ct_hline1](out\layer2_uwb_position\ct_hline1.png)

### [PASS] layer2_uwb_position          ct_hline2
- **n_points**: 435
- **kind**: line_h
- **angle_deg**: -0.07847
- **expected_deg**: 0
- **angle_err_deg**: 0.07847
- **span_m**: 0.8704
- **rms_perp_m**: 0.0196
![layer2_uwb_position ct_hline2](out\layer2_uwb_position\ct_hline2.png)

### [PASS] layer2_uwb_position          ct_hline3
- **n_points**: 438
- **kind**: line_h
- **angle_deg**: 1.542
- **expected_deg**: 0
- **angle_err_deg**: 1.542
- **span_m**: 0.8761
- **rms_perp_m**: 0.02271
![layer2_uwb_position ct_hline3](out\layer2_uwb_position\ct_hline3.png)

### [PASS] layer2_uwb_position          ct_vline1
- **n_points**: 366
- **kind**: line_v
- **angle_deg**: -96.15
- **expected_deg**: 90
- **angle_err_deg**: 6.154
- **span_m**: 0.6406
- **rms_perp_m**: 0.02108
![layer2_uwb_position ct_vline1](out\layer2_uwb_position\ct_vline1.png)

### [FAIL] layer2_uwb_position          ct_vline2
- **n_points**: 450
- **kind**: line_v
- **angle_deg**: 75.21
- **expected_deg**: 90
- **angle_err_deg**: 14.79
- **span_m**: 0.6393
- **rms_perp_m**: 0.02661
![layer2_uwb_position ct_vline2](out\layer2_uwb_position\ct_vline2.png)

### [PASS] layer2_uwb_position          ct_vline3
- **n_points**: 467
- **kind**: line_v
- **angle_deg**: -94.76
- **expected_deg**: 90
- **angle_err_deg**: 4.76
- **span_m**: 0.6248
- **rms_perp_m**: 0.02264
![layer2_uwb_position ct_vline3](out\layer2_uwb_position\ct_vline3.png)

### [PASS] layer2_uwb_position          ct_circle1
- **n_points**: 516
- **kind**: shape_circle
- **center**: [0.6137318248087779, 0.592335776988714]
- **radius_m**: 0.1063
- **rms_radial_m**: 0.02139
![layer2_uwb_position ct_circle1](out\layer2_uwb_position\ct_circle1.png)

### [PASS] layer2_uwb_position          ct_circle2
- **n_points**: 609
- **kind**: shape_circle
- **center**: [0.6331672965041951, 0.6009794759635031]
- **radius_m**: 0.1088
- **rms_radial_m**: 0.02133
![layer2_uwb_position ct_circle2](out\layer2_uwb_position\ct_circle2.png)

### [PASS] layer2_uwb_position          ct_circle3
- **n_points**: 579
- **kind**: shape_circle
- **center**: [0.6368926004948531, 0.6090601382675221]
- **radius_m**: 0.1303
- **rms_radial_m**: 0.02116
![layer2_uwb_position ct_circle3](out\layer2_uwb_position\ct_circle3.png)

### [PASS] layer2_uwb_position          ct_square1
- **n_points**: 635
- **kind**: shape_square
- **hull_area_m2**: 0.1116
- **bbox_w**: 0.4286
- **bbox_h**: 0.3087
![layer2_uwb_position ct_square1](out\layer2_uwb_position\ct_square1.png)

### [PASS] layer2_uwb_position          ct_square2
- **n_points**: 677
- **kind**: shape_square
- **hull_area_m2**: 0.1351
- **bbox_w**: 0.44
- **bbox_h**: 0.3606
![layer2_uwb_position ct_square2](out\layer2_uwb_position\ct_square2.png)

### [PASS] layer2_uwb_position          ct_square3
- **n_points**: 666
- **kind**: shape_square
- **hull_area_m2**: 0.14
- **bbox_w**: 0.4814
- **bbox_h**: 0.3465
![layer2_uwb_position ct_square3](out\layer2_uwb_position\ct_square3.png)

### [PASS] layer2_uwb_position          ct_triangle1
- **n_points**: 557
- **kind**: shape_triangle
- **hull_area_m2**: 0.1049
- **bbox_w**: 0.4673
- **bbox_h**: 0.3356
![layer2_uwb_position ct_triangle1](out\layer2_uwb_position\ct_triangle1.png)

### [PASS] layer2_uwb_position          ct_triangle2
- **n_points**: 598
- **kind**: shape_triangle
- **hull_area_m2**: 0.09384
- **bbox_w**: 0.4931
- **bbox_h**: 0.2975
![layer2_uwb_position ct_triangle2](out\layer2_uwb_position\ct_triangle2.png)

### [PASS] layer2_uwb_position          ct_triangle3
- **n_points**: 519
- **kind**: shape_triangle
- **hull_area_m2**: 0.1082
- **bbox_w**: 0.484
- **bbox_h**: 0.3754
![layer2_uwb_position ct_triangle3](out\layer2_uwb_position\ct_triangle3.png)

### [PASS] layer2_uwb_position          pt_circle1
- **n_points**: 621
- **kind**: shape_circle
- **center**: [0.6022750079815585, 0.5957690116520723]
- **radius_m**: 0.1447
- **rms_radial_m**: 0.02329
![layer2_uwb_position pt_circle1](out\layer2_uwb_position\pt_circle1.png)

### [PASS] layer2_uwb_position          pt_circle2
- **n_points**: 594
- **kind**: shape_circle
- **center**: [0.605620515941361, 0.6029478461646969]
- **radius_m**: 0.1536
- **rms_radial_m**: 0.025
![layer2_uwb_position pt_circle2](out\layer2_uwb_position\pt_circle2.png)

### [PASS] layer2_uwb_position          pt_square1
- **n_points**: 832
- **kind**: shape_square
- **hull_area_m2**: 0.1537
- **bbox_w**: 0.4769
- **bbox_h**: 0.4134
![layer2_uwb_position pt_square1](out\layer2_uwb_position\pt_square1.png)

### [PASS] layer2_uwb_position          pt_square2
- **n_points**: 732
- **kind**: shape_square
- **hull_area_m2**: 0.1515
- **bbox_w**: 0.4501
- **bbox_h**: 0.4024
![layer2_uwb_position pt_square2](out\layer2_uwb_position\pt_square2.png)

### [PASS] layer2_uwb_position          pt_triangle1
- **n_points**: 673
- **kind**: shape_triangle
- **hull_area_m2**: 0.1038
- **bbox_w**: 0.5107
- **bbox_h**: 0.3326
![layer2_uwb_position pt_triangle1](out\layer2_uwb_position\pt_triangle1.png)

### [PASS] layer2_uwb_position          pt_triangle2
- **n_points**: 649
- **kind**: shape_triangle
- **hull_area_m2**: 0.1254
- **bbox_w**: 0.5274
- **bbox_h**: 0.3684
![layer2_uwb_position pt_triangle2](out\layer2_uwb_position\pt_triangle2.png)

### [PASS] layer3_imu_raw               middle-
- **imu_count**: 3007
- **duration_s**: 18.27
- **quat_drift_deg**: 6.236
- **acc_mean_xyz**: [-0.007084935151313599, -0.0005228466910541948, 0.0002854339873628199]
- **acc_std_xyz**: [0.10987144192389724, 0.17891317617601313, 0.1434303482860126]
- **acc_noise_max_std**: 0.1789
- **deadband_ratio**: 2.236
- **above_deadband_pct**: 83.07
- **mean_imu_dt_s**: 0.006079
![layer3_imu_raw middle-](out\layer3_imu_raw\middle-.png)

### [PASS] layer4_imu_integration       middleCCW_rot-
- **n_imu**: 4632
- **category**: rotation
- **samples_until_lock**: 26
- **locked_vec**: [0.9951144227025444, -0.09872834309043102]
- **locked_heading_deg**: -5.666
- **post_lock_drift_deg**: 9.334
- **tag_z_error_mean_m**: 0.0002076
- **tag_z_error_max_m**: 0.001626
- **dr_drift_mean_m**: 0.103
- **dr_drift_max_m**: 0.5182
- **mean_imu_dt_s**: 0.005802
- **mean_omega_rad_s**: 0.32
- **max_omega_rad_s**: 6.534
- **lever_arm_active_frac**: 0.07124
- **position_scatter_R95**: 0.2384
- **heading_range_deg**: 13.49
- **heading_drift_rate_deg_per_s**: 0.5019
![layer4_imu_integration middleCCW_rot-](out\layer4_imu_integration\middleCCW_rot-.png)

### [FAIL] layer4_imu_integration       middleCW_rot-
- **n_imu**: 5014
- **category**: rotation
- **samples_until_lock**: 32
- **locked_vec**: [0.9999818717215324, -0.006021314499386495]
- **locked_heading_deg**: -0.345
- **post_lock_drift_deg**: 15.31
- **tag_z_error_mean_m**: 0.0002609
- **tag_z_error_max_m**: 0.001363
- **dr_drift_mean_m**: 0.1206
- **dr_drift_max_m**: 0.5284
- **mean_imu_dt_s**: 0.005772
- **mean_omega_rad_s**: 0.2954
- **max_omega_rad_s**: 7.002
- **lever_arm_active_frac**: 0.05465
- **position_scatter_R95**: 0.1297
- **heading_range_deg**: 22.22
- **heading_drift_rate_deg_per_s**: 0.7679
![layer4_imu_integration middleCW_rot-](out\layer4_imu_integration\middleCW_rot-.png)

### [PASS] layer4_imu_integration       rt_circle
- **n_imu**: 3279
- **category**: rotation
- **samples_until_lock**: 42
- **locked_vec**: [0.9999681686806503, 0.007978823564054001]
- **locked_heading_deg**: 0.4572
- **post_lock_drift_deg**: 8.137
- **tag_z_error_mean_m**: 0.0009232
- **tag_z_error_max_m**: 0.002872
- **dr_drift_mean_m**: 0.2258
- **dr_drift_max_m**: 1.239
- **mean_imu_dt_s**: 0.005639
- **mean_omega_rad_s**: 0.2688
- **max_omega_rad_s**: 3.187
- **lever_arm_active_frac**: 0.03111
- **position_scatter_R95**: 0.2251
- **heading_range_deg**: 11.21
- **heading_drift_rate_deg_per_s**: 0.6065
![layer4_imu_integration rt_circle](out\layer4_imu_integration\rt_circle.png)

### [PASS] layer4_imu_integration       rt_square
- **n_imu**: 3291
- **category**: rotation
- **samples_until_lock**: 31
- **locked_vec**: [0.9979123687520017, -0.06458253859805241]
- **locked_heading_deg**: -3.703
- **post_lock_drift_deg**: 5.603
- **tag_z_error_mean_m**: 0.0002018
- **tag_z_error_max_m**: 0.0008958
- **dr_drift_mean_m**: 0.3083
- **dr_drift_max_m**: 1.193
- **mean_imu_dt_s**: 0.00575
- **mean_omega_rad_s**: 0.2651
- **max_omega_rad_s**: 2.966
- **lever_arm_active_frac**: 0.02613
- **position_scatter_R95**: 0.2451
- **heading_range_deg**: 7.986
- **heading_drift_rate_deg_per_s**: 0.422
![layer4_imu_integration rt_square](out\layer4_imu_integration\rt_square.png)

### [PASS] layer4_imu_integration       rt_triangle
- **n_imu**: 3108
- **category**: rotation
- **samples_until_lock**: 39
- **locked_vec**: [0.9984979839684845, -0.05478846603868412]
- **locked_heading_deg**: -3.141
- **post_lock_drift_deg**: 5.87
- **tag_z_error_mean_m**: 0.0003996
- **tag_z_error_max_m**: 0.002248
- **dr_drift_mean_m**: 0.2063
- **dr_drift_max_m**: 0.8355
- **mean_imu_dt_s**: 0.005802
- **mean_omega_rad_s**: 0.2813
- **max_omega_rad_s**: 2.647
- **lever_arm_active_frac**: 0.02992
- **position_scatter_R95**: 0.2493
- **heading_range_deg**: 7.468
- **heading_drift_rate_deg_per_s**: 0.4141
![layer4_imu_integration rt_triangle](out\layer4_imu_integration\rt_triangle.png)

### [FAIL] layer4_imu_integration       ct_circle1
- **n_imu**: 1800
- **category**: motion
- **samples_until_lock**: 26
- **locked_vec**: [0.9885739186631313, 0.1507368811506354]
- **locked_heading_deg**: 8.67
- **post_lock_drift_deg**: 6.329
- **tag_z_error_mean_m**: 0.0005781
- **tag_z_error_max_m**: 0.001854
- **dr_drift_mean_m**: 0.1281
- **dr_drift_max_m**: 1.083
- **mean_imu_dt_s**: 0.006279
- **mean_omega_rad_s**: 0.2115
- **max_omega_rad_s**: 3.704
- **lever_arm_active_frac**: 0.02444
- **motion_angle_deg**: -90.15
- **heading_vs_motion_deg**: 81.19
![layer4_imu_integration ct_circle1](out\layer4_imu_integration\ct_circle1.png)

### [FAIL] layer4_imu_integration       ct_circle2
- **n_imu**: 2131
- **category**: motion
- **samples_until_lock**: 30
- **locked_vec**: [0.9878565525504225, 0.15536869563459202]
- **locked_heading_deg**: 8.938
- **post_lock_drift_deg**: 6.062
- **tag_z_error_mean_m**: 0.0005454
- **tag_z_error_max_m**: 0.001727
- **dr_drift_mean_m**: 0.1372
- **dr_drift_max_m**: 0.8759
- **mean_imu_dt_s**: 0.005803
- **mean_omega_rad_s**: 0.1913
- **max_omega_rad_s**: 2.081
- **lever_arm_active_frac**: 0.01689
- **motion_angle_deg**: -87.53
- **heading_vs_motion_deg**: 83.54
![layer4_imu_integration ct_circle2](out\layer4_imu_integration\ct_circle2.png)

### [FAIL] layer4_imu_integration       ct_circle3
- **n_imu**: 2039
- **category**: motion
- **samples_until_lock**: 26
- **locked_vec**: [0.9999821378906787, 0.0059769473468932926]
- **locked_heading_deg**: 0.3425
- **post_lock_drift_deg**: 8.284
- **tag_z_error_mean_m**: 0.0002728
- **tag_z_error_max_m**: 0.001192
- **dr_drift_mean_m**: 0.1073
- **dr_drift_max_m**: 0.509
- **mean_imu_dt_s**: 0.005761
- **mean_omega_rad_s**: 0.1954
- **max_omega_rad_s**: 2.531
- **lever_arm_active_frac**: 0.01422
- **motion_angle_deg**: -75.09
- **heading_vs_motion_deg**: 75.43
![layer4_imu_integration ct_circle3](out\layer4_imu_integration\ct_circle3.png)

### [FAIL] layer4_imu_integration       ct_diagonal1
- **n_imu**: 1833
- **category**: motion_line
- **samples_until_lock**: 30
- **locked_vec**: [0.9825173396791859, -0.1861710966550267]
- **locked_heading_deg**: -10.73
- **post_lock_drift_deg**: 4.059
- **tag_z_error_mean_m**: 0.0009073
- **tag_z_error_max_m**: 0.001608
- **dr_drift_mean_m**: 0.1119
- **dr_drift_max_m**: 0.7706
- **mean_imu_dt_s**: 0.005688
- **mean_omega_rad_s**: 0.1673
- **max_omega_rad_s**: 1.758
- **lever_arm_active_frac**: 0.01964
- **motion_angle_deg**: -38.15
- **heading_vs_motion_deg**: 27.42
![layer4_imu_integration ct_diagonal1](out\layer4_imu_integration\ct_diagonal1.png)

### [FAIL] layer4_imu_integration       ct_diagonal2
- **n_imu**: 1564
- **category**: motion_line
- **samples_until_lock**: 32
- **locked_vec**: [0.991454134884777, -0.1304557335646025]
- **locked_heading_deg**: -7.496
- **post_lock_drift_deg**: 6.459
- **tag_z_error_mean_m**: 0.0006594
- **tag_z_error_max_m**: 0.002079
- **dr_drift_mean_m**: 0.1394
- **dr_drift_max_m**: 0.9278
- **mean_imu_dt_s**: 0.005715
- **mean_omega_rad_s**: 0.1925
- **max_omega_rad_s**: 2.05
- **lever_arm_active_frac**: 0.01982
- **motion_angle_deg**: -40.22
- **heading_vs_motion_deg**: 32.73
![layer4_imu_integration ct_diagonal2](out\layer4_imu_integration\ct_diagonal2.png)

### [FAIL] layer4_imu_integration       ct_diagonal3
- **n_imu**: 1731
- **category**: motion_line
- **samples_until_lock**: 32
- **locked_vec**: [0.9900674645114238, -0.14059308559072423]
- **locked_heading_deg**: -8.082
- **post_lock_drift_deg**: 5.073
- **tag_z_error_mean_m**: 0.000765
- **tag_z_error_max_m**: 0.001674
- **dr_drift_mean_m**: 0.1895
- **dr_drift_max_m**: 1.398
- **mean_imu_dt_s**: 0.005857
- **mean_omega_rad_s**: 0.1922
- **max_omega_rad_s**: 3.065
- **lever_arm_active_frac**: 0.0156
- **motion_angle_deg**: -39.17
- **heading_vs_motion_deg**: 31.08
![layer4_imu_integration ct_diagonal3](out\layer4_imu_integration\ct_diagonal3.png)

### [FAIL] layer4_imu_integration       ct_hline1
- **n_imu**: 1538
- **category**: motion_line
- **samples_until_lock**: 27
- **locked_vec**: [0.9970959564881938, -0.07615545650111874]
- **locked_heading_deg**: -4.368
- **post_lock_drift_deg**: 3.448
- **tag_z_error_mean_m**: 0.0004341
- **tag_z_error_max_m**: 0.00162
- **dr_drift_mean_m**: 0.1206
- **dr_drift_max_m**: 0.6626
- **mean_imu_dt_s**: 0.005599
- **mean_omega_rad_s**: 0.2258
- **max_omega_rad_s**: 1.851
- **lever_arm_active_frac**: 0.01625
- **motion_angle_deg**: 0.2241
- **heading_vs_motion_deg**: 4.592
![layer4_imu_integration ct_hline1](out\layer4_imu_integration\ct_hline1.png)

### [FAIL] layer4_imu_integration       ct_hline2
- **n_imu**: 1565
- **category**: motion_line
- **samples_until_lock**: 28
- **locked_vec**: [0.9967808536340337, -0.08017437139514831]
- **locked_heading_deg**: -4.599
- **post_lock_drift_deg**: 4.667
- **tag_z_error_mean_m**: 0.0002957
- **tag_z_error_max_m**: 0.001722
- **dr_drift_mean_m**: 0.1695
- **dr_drift_max_m**: 1.369
- **mean_imu_dt_s**: 0.005787
- **mean_omega_rad_s**: 0.1802
- **max_omega_rad_s**: 1.905
- **lever_arm_active_frac**: 0.01789
- **motion_angle_deg**: -0.07847
- **heading_vs_motion_deg**: 4.52
![layer4_imu_integration ct_hline2](out\layer4_imu_integration\ct_hline2.png)

### [FAIL] layer4_imu_integration       ct_hline3
- **n_imu**: 1526
- **category**: motion_line
- **samples_until_lock**: 31
- **locked_vec**: [0.9912541781441216, -0.13196648935173586]
- **locked_heading_deg**: -7.583
- **post_lock_drift_deg**: 5.355
- **tag_z_error_mean_m**: 0.0005936
- **tag_z_error_max_m**: 0.003231
- **dr_drift_mean_m**: 0.1148
- **dr_drift_max_m**: 0.601
- **mean_imu_dt_s**: 0.006836
- **mean_omega_rad_s**: 0.1733
- **max_omega_rad_s**: 3.053
- **lever_arm_active_frac**: 0.02359
- **motion_angle_deg**: 1.542
- **heading_vs_motion_deg**: 9.125
![layer4_imu_integration ct_hline3](out\layer4_imu_integration\ct_hline3.png)

### [FAIL] layer4_imu_integration       ct_square1
- **n_imu**: 2187
- **category**: motion
- **samples_until_lock**: 29
- **locked_vec**: [0.9993830857022449, -0.03512047853118556]
- **locked_heading_deg**: -2.013
- **post_lock_drift_deg**: 4.813
- **tag_z_error_mean_m**: 0.0002524
- **tag_z_error_max_m**: 0.00155
- **dr_drift_mean_m**: 0.1282
- **dr_drift_max_m**: 0.7274
- **mean_imu_dt_s**: 0.00585
- **mean_omega_rad_s**: 0.2005
- **max_omega_rad_s**: 1.935
- **lever_arm_active_frac**: 0.01372
- **motion_angle_deg**: -18.55
- **heading_vs_motion_deg**: 16.53
![layer4_imu_integration ct_square1](out\layer4_imu_integration\ct_square1.png)

### [FAIL] layer4_imu_integration       ct_square2
- **n_imu**: 2323
- **category**: motion
- **samples_until_lock**: 26
- **locked_vec**: [0.9999309859923354, -0.011748329770472915]
- **locked_heading_deg**: -0.6731
- **post_lock_drift_deg**: 4.737
- **tag_z_error_mean_m**: 0.0003968
- **tag_z_error_max_m**: 0.003184
- **dr_drift_mean_m**: 0.1145
- **dr_drift_max_m**: 0.7835
- **mean_imu_dt_s**: 0.005948
- **mean_omega_rad_s**: 0.1895
- **max_omega_rad_s**: 2.072
- **lever_arm_active_frac**: 0.0155
- **motion_angle_deg**: -16.48
- **heading_vs_motion_deg**: 15.81
![layer4_imu_integration ct_square2](out\layer4_imu_integration\ct_square2.png)

### [FAIL] layer4_imu_integration       ct_square3
- **n_imu**: 2334
- **category**: motion
- **samples_until_lock**: 34
- **locked_vec**: [0.9981528539259612, -0.060752614754090536]
- **locked_heading_deg**: -3.483
- **post_lock_drift_deg**: 7.275
- **tag_z_error_mean_m**: 0.0002401
- **tag_z_error_max_m**: 0.002461
- **dr_drift_mean_m**: 0.09049
- **dr_drift_max_m**: 0.5795
- **mean_imu_dt_s**: 0.005911
- **mean_omega_rad_s**: 0.2001
- **max_omega_rad_s**: 3.439
- **lever_arm_active_frac**: 0.02271
- **motion_angle_deg**: -14.1
- **heading_vs_motion_deg**: 10.61
![layer4_imu_integration ct_square3](out\layer4_imu_integration\ct_square3.png)

### [FAIL] layer4_imu_integration       ct_triangle1
- **n_imu**: 1943
- **category**: motion
- **samples_until_lock**: 28
- **locked_vec**: [0.9994600360857006, 0.03285781897159761]
- **locked_heading_deg**: 1.883
- **post_lock_drift_deg**: 4.821
- **tag_z_error_mean_m**: 0.0003225
- **tag_z_error_max_m**: 0.002023
- **dr_drift_mean_m**: 0.1404
- **dr_drift_max_m**: 0.8349
- **mean_imu_dt_s**: 0.005836
- **mean_omega_rad_s**: 0.2107
- **max_omega_rad_s**: 2.49
- **lever_arm_active_frac**: 0.02316
- **motion_angle_deg**: -66.6
- **heading_vs_motion_deg**: 68.48
![layer4_imu_integration ct_triangle1](out\layer4_imu_integration\ct_triangle1.png)

### [FAIL] layer4_imu_integration       ct_triangle2
- **n_imu**: 2136
- **category**: motion
- **samples_until_lock**: 26
- **locked_vec**: [0.9989611769987108, 0.04556936305622951]
- **locked_heading_deg**: 2.612
- **post_lock_drift_deg**: 6.246
- **tag_z_error_mean_m**: 0.0003405
- **tag_z_error_max_m**: 0.001684
- **dr_drift_mean_m**: 0.105
- **dr_drift_max_m**: 0.7659
- **mean_imu_dt_s**: 0.005705
- **mean_omega_rad_s**: 0.1825
- **max_omega_rad_s**: 2.483
- **lever_arm_active_frac**: 0.01732
- **motion_angle_deg**: -70.52
- **heading_vs_motion_deg**: 73.13
![layer4_imu_integration ct_triangle2](out\layer4_imu_integration\ct_triangle2.png)

### [FAIL] layer4_imu_integration       ct_triangle3
- **n_imu**: 1833
- **category**: motion
- **samples_until_lock**: 28
- **locked_vec**: [0.999144762582272, 0.04134904357316456]
- **locked_heading_deg**: 2.37
- **post_lock_drift_deg**: 7.532
- **tag_z_error_mean_m**: 0.0004353
- **tag_z_error_max_m**: 0.001681
- **dr_drift_mean_m**: 0.2315
- **dr_drift_max_m**: 1.58
- **mean_imu_dt_s**: 0.005758
- **mean_omega_rad_s**: 0.2351
- **max_omega_rad_s**: 2.248
- **lever_arm_active_frac**: 0.03764
- **motion_angle_deg**: -99.05
- **heading_vs_motion_deg**: 78.58
![layer4_imu_integration ct_triangle3](out\layer4_imu_integration\ct_triangle3.png)

### [FAIL] layer4_imu_integration       ct_vline1
- **n_imu**: 1251
- **category**: motion_line
- **samples_until_lock**: 28
- **locked_vec**: [0.9999685738210918, -0.007927885607890124]
- **locked_heading_deg**: -0.4542
- **post_lock_drift_deg**: 6.118
- **tag_z_error_mean_m**: 0.001823
- **tag_z_error_max_m**: 0.00252
- **dr_drift_mean_m**: 0.09752
- **dr_drift_max_m**: 0.6677
- **mean_imu_dt_s**: 0.006094
- **mean_omega_rad_s**: 0.183
- **max_omega_rad_s**: 2.135
- **lever_arm_active_frac**: 0.009592
- **motion_angle_deg**: -96.15
- **heading_vs_motion_deg**: 84.3
![layer4_imu_integration ct_vline1](out\layer4_imu_integration\ct_vline1.png)

### [FAIL] layer4_imu_integration       ct_vline2
- **n_imu**: 1550
- **category**: motion_line
- **samples_until_lock**: 29
- **locked_vec**: [0.99996953115368, -0.007806200374653078]
- **locked_heading_deg**: -0.4473
- **post_lock_drift_deg**: 5.407
- **tag_z_error_mean_m**: 0.001844
- **tag_z_error_max_m**: 0.004554
- **dr_drift_mean_m**: 0.07112
- **dr_drift_max_m**: 0.38
- **mean_imu_dt_s**: 0.006056
- **mean_omega_rad_s**: 0.1555
- **max_omega_rad_s**: 2.919
- **lever_arm_active_frac**: 0.01032
- **motion_angle_deg**: 75.21
- **heading_vs_motion_deg**: 75.66
![layer4_imu_integration ct_vline2](out\layer4_imu_integration\ct_vline2.png)

### [FAIL] layer4_imu_integration       ct_vline3
- **n_imu**: 1650
- **category**: motion_line
- **samples_until_lock**: 27
- **locked_vec**: [0.9999999497012065, -0.00031717122285402895]
- **locked_heading_deg**: -0.01817
- **post_lock_drift_deg**: 4.796
- **tag_z_error_mean_m**: 0.002051
- **tag_z_error_max_m**: 0.005484
- **dr_drift_mean_m**: 0.1122
- **dr_drift_max_m**: 0.6106
- **mean_imu_dt_s**: 0.005896
- **mean_omega_rad_s**: 0.1609
- **max_omega_rad_s**: 1.864
- **lever_arm_active_frac**: 0.01697
- **motion_angle_deg**: -94.76
- **heading_vs_motion_deg**: 85.26
![layer4_imu_integration ct_vline3](out\layer4_imu_integration\ct_vline3.png)

### [FAIL] layer4_imu_integration       pt_circle1
- **n_imu**: 2221
- **category**: motion
- **samples_until_lock**: 33
- **locked_vec**: [0.9983168386060681, -0.05799560117444915]
- **locked_heading_deg**: -3.325
- **post_lock_drift_deg**: 6.044
- **tag_z_error_mean_m**: 0.001298
- **tag_z_error_max_m**: 0.003384
- **dr_drift_mean_m**: 0.1896
- **dr_drift_max_m**: 1.067
- **mean_imu_dt_s**: 0.005679
- **mean_omega_rad_s**: 0.2415
- **max_omega_rad_s**: 2.837
- **lever_arm_active_frac**: 0.03332
- **motion_angle_deg**: -92.42
- **heading_vs_motion_deg**: 89.1
![layer4_imu_integration pt_circle1](out\layer4_imu_integration\pt_circle1.png)

### [FAIL] layer4_imu_integration       pt_circle2
- **n_imu**: 2136
- **category**: motion
- **samples_until_lock**: 29
- **locked_vec**: [0.9962182243130184, -0.08688641750363783]
- **locked_heading_deg**: -4.985
- **post_lock_drift_deg**: 8.897
- **tag_z_error_mean_m**: 0.0009721
- **tag_z_error_max_m**: 0.002594
- **dr_drift_mean_m**: 0.1617
- **dr_drift_max_m**: 0.8158
- **mean_imu_dt_s**: 0.005758
- **mean_omega_rad_s**: 0.272
- **max_omega_rad_s**: 2.577
- **lever_arm_active_frac**: 0.04541
- **motion_angle_deg**: -85.43
- **heading_vs_motion_deg**: 80.44
![layer4_imu_integration pt_circle2](out\layer4_imu_integration\pt_circle2.png)

### [FAIL] layer4_imu_integration       pt_square1
- **n_imu**: 2785
- **category**: motion
- **samples_until_lock**: 26
- **locked_vec**: [0.9995171974391356, -0.03107043648576778]
- **locked_heading_deg**: -1.78
- **post_lock_drift_deg**: 8.576
- **tag_z_error_mean_m**: 0.001303
- **tag_z_error_max_m**: 0.004221
- **dr_drift_mean_m**: 0.1104
- **dr_drift_max_m**: 0.5883
- **mean_imu_dt_s**: 0.00575
- **mean_omega_rad_s**: 0.2338
- **max_omega_rad_s**: 2.826
- **lever_arm_active_frac**: 0.03124
- **motion_angle_deg**: -31.66
- **heading_vs_motion_deg**: 29.88
![layer4_imu_integration pt_square1](out\layer4_imu_integration\pt_square1.png)

### [FAIL] layer4_imu_integration       pt_square2
- **n_imu**: 2604
- **category**: motion
- **samples_until_lock**: 32
- **locked_vec**: [0.9951592094374823, -0.0982758763469728]
- **locked_heading_deg**: -5.64
- **post_lock_drift_deg**: 6.276
- **tag_z_error_mean_m**: 0.0009243
- **tag_z_error_max_m**: 0.002768
- **dr_drift_mean_m**: 0.2128
- **dr_drift_max_m**: 1.15
- **mean_imu_dt_s**: 0.005701
- **mean_omega_rad_s**: 0.2507
- **max_omega_rad_s**: 4.802
- **lever_arm_active_frac**: 0.0384
- **motion_angle_deg**: -30.74
- **heading_vs_motion_deg**: 25.1
![layer4_imu_integration pt_square2](out\layer4_imu_integration\pt_square2.png)

### [FAIL] layer4_imu_integration       pt_triangle1
- **n_imu**: 2443
- **category**: motion
- **samples_until_lock**: 31
- **locked_vec**: [0.9993299214864333, -0.03660202210150705]
- **locked_heading_deg**: -2.098
- **post_lock_drift_deg**: 7.66
- **tag_z_error_mean_m**: 0.0006153
- **tag_z_error_max_m**: 0.002234
- **dr_drift_mean_m**: 0.2371
- **dr_drift_max_m**: 1.278
- **mean_imu_dt_s**: 0.005598
- **mean_omega_rad_s**: 0.1971
- **max_omega_rad_s**: 2.627
- **lever_arm_active_frac**: 0.01678
- **motion_angle_deg**: -158
- **heading_vs_motion_deg**: 24.15
![layer4_imu_integration pt_triangle1](out\layer4_imu_integration\pt_triangle1.png)

### [FAIL] layer4_imu_integration       pt_triangle2
- **n_imu**: 2351
- **category**: motion
- **samples_until_lock**: 29
- **locked_vec**: [0.9996349309346458, -0.02701860202315969]
- **locked_heading_deg**: -1.548
- **post_lock_drift_deg**: 8.065
- **tag_z_error_mean_m**: 0.0007129
- **tag_z_error_max_m**: 0.002315
- **dr_drift_mean_m**: 0.2248
- **dr_drift_max_m**: 1.24
- **mean_imu_dt_s**: 0.005603
- **mean_omega_rad_s**: 0.2086
- **max_omega_rad_s**: 2.425
- **lever_arm_active_frac**: 0.01829
- **motion_angle_deg**: -120.2
- **heading_vs_motion_deg**: 61.4
![layer4_imu_integration pt_triangle2](out\layer4_imu_integration\pt_triangle2.png)

### [PASS] layer5_force_contact         ABC_b1
- **imu_count**: 3851
- **raw_min**: 0
- **raw_max**: 2282
- **raw_median**: 0
- **up_transitions**: 5
- **down_transitions**: 5
- **contact_episodes**: 5
- **mean_episode_samples**: 384.8
- **min_episode_samples**: 190
- **glitch_count**: 0
- **contact_duty_pct**: 49.96
![layer5_force_contact ABC_b1](out\layer5_force_contact\ABC_b1.png)

### [PASS] layer5_force_contact         ABC_b2
- **imu_count**: 3885
- **raw_min**: 0
- **raw_max**: 2317
- **raw_median**: 0
- **up_transitions**: 5
- **down_transitions**: 5
- **contact_episodes**: 5
- **mean_episode_samples**: 354.2
- **min_episode_samples**: 142
- **glitch_count**: 0
- **contact_duty_pct**: 45.59
![layer5_force_contact ABC_b2](out\layer5_force_contact\ABC_b2.png)

### [PASS] layer5_force_contact         abc_s1
- **imu_count**: 3203
- **raw_min**: 0
- **raw_max**: 2274
- **raw_median**: 0
- **up_transitions**: 3
- **down_transitions**: 3
- **contact_episodes**: 3
- **mean_episode_samples**: 494.3
- **min_episode_samples**: 330
- **glitch_count**: 0
- **contact_duty_pct**: 46.3
![layer5_force_contact abc_s1](out\layer5_force_contact\abc_s1.png)

### [PASS] layer5_force_contact         abc_s2
- **imu_count**: 2825
- **raw_min**: 0
- **raw_max**: 2281
- **raw_median**: 0
- **up_transitions**: 3
- **down_transitions**: 3
- **contact_episodes**: 3
- **mean_episode_samples**: 451.7
- **min_episode_samples**: 351
- **glitch_count**: 0
- **contact_duty_pct**: 47.96
![layer5_force_contact abc_s2](out\layer5_force_contact\abc_s2.png)

### [PASS] layer5_force_contact         ct_circle1
- **imu_count**: 1800
- **raw_min**: 0
- **raw_max**: 2251
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 664
- **min_episode_samples**: 664
- **glitch_count**: 0
- **contact_duty_pct**: 36.89
![layer5_force_contact ct_circle1](out\layer5_force_contact\ct_circle1.png)

### [PASS] layer5_force_contact         ct_circle2
- **imu_count**: 2131
- **raw_min**: 0
- **raw_max**: 2248
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 712
- **min_episode_samples**: 712
- **glitch_count**: 0
- **contact_duty_pct**: 33.41
![layer5_force_contact ct_circle2](out\layer5_force_contact\ct_circle2.png)

### [PASS] layer5_force_contact         ct_circle3
- **imu_count**: 2039
- **raw_min**: 0
- **raw_max**: 2268
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 822
- **min_episode_samples**: 822
- **glitch_count**: 0
- **contact_duty_pct**: 40.31
![layer5_force_contact ct_circle3](out\layer5_force_contact\ct_circle3.png)

### [PASS] layer5_force_contact         ct_diagonal1
- **imu_count**: 1833
- **raw_min**: 0
- **raw_max**: 2251
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 626
- **min_episode_samples**: 626
- **glitch_count**: 0
- **contact_duty_pct**: 34.15
![layer5_force_contact ct_diagonal1](out\layer5_force_contact\ct_diagonal1.png)

### [PASS] layer5_force_contact         ct_diagonal2
- **imu_count**: 1564
- **raw_min**: 0
- **raw_max**: 2397
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 716
- **min_episode_samples**: 716
- **glitch_count**: 0
- **contact_duty_pct**: 45.78
![layer5_force_contact ct_diagonal2](out\layer5_force_contact\ct_diagonal2.png)

### [PASS] layer5_force_contact         ct_diagonal3
- **imu_count**: 1731
- **raw_min**: 0
- **raw_max**: 2336
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 669
- **min_episode_samples**: 669
- **glitch_count**: 0
- **contact_duty_pct**: 38.65
![layer5_force_contact ct_diagonal3](out\layer5_force_contact\ct_diagonal3.png)

### [PASS] layer5_force_contact         ct_hline1
- **imu_count**: 1538
- **raw_min**: 0
- **raw_max**: 2296
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 506
- **min_episode_samples**: 506
- **glitch_count**: 0
- **contact_duty_pct**: 32.9
![layer5_force_contact ct_hline1](out\layer5_force_contact\ct_hline1.png)

### [PASS] layer5_force_contact         ct_hline2
- **imu_count**: 1565
- **raw_min**: 0
- **raw_max**: 2396
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 448
- **min_episode_samples**: 448
- **glitch_count**: 0
- **contact_duty_pct**: 28.63
![layer5_force_contact ct_hline2](out\layer5_force_contact\ct_hline2.png)

### [PASS] layer5_force_contact         ct_hline3
- **imu_count**: 1526
- **raw_min**: 0
- **raw_max**: 2441
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 455
- **min_episode_samples**: 455
- **glitch_count**: 0
- **contact_duty_pct**: 29.82
![layer5_force_contact ct_hline3](out\layer5_force_contact\ct_hline3.png)

### [PASS] layer5_force_contact         ct_square1
- **imu_count**: 2187
- **raw_min**: 0
- **raw_max**: 2336
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 1080
- **min_episode_samples**: 1080
- **glitch_count**: 0
- **contact_duty_pct**: 49.38
![layer5_force_contact ct_square1](out\layer5_force_contact\ct_square1.png)

### [PASS] layer5_force_contact         ct_square2
- **imu_count**: 2323
- **raw_min**: 0
- **raw_max**: 2361
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 1099
- **min_episode_samples**: 1099
- **glitch_count**: 0
- **contact_duty_pct**: 47.31
![layer5_force_contact ct_square2](out\layer5_force_contact\ct_square2.png)

### [PASS] layer5_force_contact         ct_square3
- **imu_count**: 2334
- **raw_min**: 0
- **raw_max**: 2343
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 1091
- **min_episode_samples**: 1091
- **glitch_count**: 0
- **contact_duty_pct**: 46.74
![layer5_force_contact ct_square3](out\layer5_force_contact\ct_square3.png)

### [PASS] layer5_force_contact         ct_triangle1
- **imu_count**: 1943
- **raw_min**: 0
- **raw_max**: 2319
- **raw_median**: 0
- **up_transitions**: 2
- **down_transitions**: 2
- **contact_episodes**: 2
- **mean_episode_samples**: 459
- **min_episode_samples**: 179
- **glitch_count**: 0
- **contact_duty_pct**: 47.25
![layer5_force_contact ct_triangle1](out\layer5_force_contact\ct_triangle1.png)

### [PASS] layer5_force_contact         ct_triangle2
- **imu_count**: 2136
- **raw_min**: 0
- **raw_max**: 2344
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 951
- **min_episode_samples**: 951
- **glitch_count**: 0
- **contact_duty_pct**: 44.52
![layer5_force_contact ct_triangle2](out\layer5_force_contact\ct_triangle2.png)

### [PASS] layer5_force_contact         ct_triangle3
- **imu_count**: 1833
- **raw_min**: 0
- **raw_max**: 2359
- **raw_median**: 0
- **up_transitions**: 2
- **down_transitions**: 2
- **contact_episodes**: 2
- **mean_episode_samples**: 390
- **min_episode_samples**: 145
- **glitch_count**: 0
- **contact_duty_pct**: 42.55
![layer5_force_contact ct_triangle3](out\layer5_force_contact\ct_triangle3.png)

### [PASS] layer5_force_contact         ct_vline1
- **imu_count**: 1251
- **raw_min**: 0
- **raw_max**: 2255
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 433
- **min_episode_samples**: 433
- **glitch_count**: 0
- **contact_duty_pct**: 34.61
![layer5_force_contact ct_vline1](out\layer5_force_contact\ct_vline1.png)

### [PASS] layer5_force_contact         ct_vline2
- **imu_count**: 1550
- **raw_min**: 0
- **raw_max**: 2330
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 475
- **min_episode_samples**: 475
- **glitch_count**: 0
- **contact_duty_pct**: 30.65
![layer5_force_contact ct_vline2](out\layer5_force_contact\ct_vline2.png)

### [PASS] layer5_force_contact         ct_vline3
- **imu_count**: 1650
- **raw_min**: 0
- **raw_max**: 2299
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 488
- **min_episode_samples**: 488
- **glitch_count**: 0
- **contact_duty_pct**: 29.58
![layer5_force_contact ct_vline3](out\layer5_force_contact\ct_vline3.png)

### [PASS] layer5_force_contact         HELLO_b1
- **imu_count**: 5239
- **raw_min**: 0
- **raw_max**: 2344
- **raw_median**: 0
- **up_transitions**: 9
- **down_transitions**: 9
- **contact_episodes**: 9
- **mean_episode_samples**: 252.4
- **min_episode_samples**: 116
- **glitch_count**: 0
- **contact_duty_pct**: 43.37
![layer5_force_contact HELLO_b1](out\layer5_force_contact\HELLO_b1.png)

### [PASS] layer5_force_contact         HELLO_b2
- **imu_count**: 5150
- **raw_min**: 0
- **raw_max**: 2340
- **raw_median**: 0
- **up_transitions**: 9
- **down_transitions**: 9
- **contact_episodes**: 9
- **mean_episode_samples**: 277.3
- **min_episode_samples**: 120
- **glitch_count**: 0
- **contact_duty_pct**: 48.47
![layer5_force_contact HELLO_b2](out\layer5_force_contact\HELLO_b2.png)

### [PASS] layer5_force_contact         hello_s1
- **imu_count**: 3075
- **raw_min**: 0
- **raw_max**: 2317
- **raw_median**: 0
- **up_transitions**: 5
- **down_transitions**: 5
- **contact_episodes**: 5
- **mean_episode_samples**: 271
- **min_episode_samples**: 155
- **glitch_count**: 0
- **contact_duty_pct**: 44.07
![layer5_force_contact hello_s1](out\layer5_force_contact\hello_s1.png)

### [PASS] layer5_force_contact         hello_s2
- **imu_count**: 2913
- **raw_min**: 0
- **raw_max**: 2377
- **raw_median**: 0
- **up_transitions**: 5
- **down_transitions**: 5
- **contact_episodes**: 5
- **mean_episode_samples**: 257.8
- **min_episode_samples**: 131
- **glitch_count**: 0
- **contact_duty_pct**: 44.25
![layer5_force_contact hello_s2](out\layer5_force_contact\hello_s2.png)

### [PASS] layer5_force_contact         imu_a
- **imu_count**: 1539
- **raw_min**: 0
- **raw_max**: 2045
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 259
- **min_episode_samples**: 259
- **glitch_count**: 0
- **contact_duty_pct**: 16.83
![layer5_force_contact imu_a](out\layer5_force_contact\imu_a.png)

### [PASS] layer5_force_contact         imu_circle
- **imu_count**: 1355
- **raw_min**: 0
- **raw_max**: 2115
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 236
- **min_episode_samples**: 236
- **glitch_count**: 0
- **contact_duty_pct**: 17.42
![layer5_force_contact imu_circle](out\layer5_force_contact\imu_circle.png)

### [PASS] layer5_force_contact         imu_square
- **imu_count**: 1440
- **raw_min**: 0
- **raw_max**: 2130
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 476
- **min_episode_samples**: 476
- **glitch_count**: 0
- **contact_duty_pct**: 33.06
![layer5_force_contact imu_square](out\layer5_force_contact\imu_square.png)

### [PASS] layer5_force_contact         imu_triangle
- **imu_count**: 1559
- **raw_min**: 0
- **raw_max**: 2200
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 355
- **min_episode_samples**: 355
- **glitch_count**: 0
- **contact_duty_pct**: 22.77
![layer5_force_contact imu_triangle](out\layer5_force_contact\imu_triangle.png)

### [PASS] layer5_force_contact         middle-
- **imu_count**: 3007
- **raw_min**: 0
- **raw_max**: 2293
- **raw_median**: 2207
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 2390
- **min_episode_samples**: 2390
- **glitch_count**: 0
- **contact_duty_pct**: 79.48
![layer5_force_contact middle-](out\layer5_force_contact\middle-.png)

### [PASS] layer5_force_contact         middleCCW_rot-
- **imu_count**: 4632
- **raw_min**: 2182
- **raw_max**: 2327
- **raw_median**: 2265
- **up_transitions**: 1
- **down_transitions**: 0
- **contact_episodes**: 1
- **mean_episode_samples**: 4630
- **min_episode_samples**: 4630
- **glitch_count**: 0
- **contact_duty_pct**: 99.96
![layer5_force_contact middleCCW_rot-](out\layer5_force_contact\middleCCW_rot-.png)

### [PASS] layer5_force_contact         middleCW_rot-
- **imu_count**: 5014
- **raw_min**: 0
- **raw_max**: 2348
- **raw_median**: 2266
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 4986
- **min_episode_samples**: 4986
- **glitch_count**: 0
- **contact_duty_pct**: 99.44
![layer5_force_contact middleCW_rot-](out\layer5_force_contact\middleCW_rot-.png)

### [PASS] layer5_force_contact         pt_a1
- **imu_count**: 1822
- **raw_min**: 0
- **raw_max**: 2303
- **raw_median**: 0
- **up_transitions**: 2
- **down_transitions**: 2
- **contact_episodes**: 2
- **mean_episode_samples**: 261
- **min_episode_samples**: 148
- **glitch_count**: 0
- **contact_duty_pct**: 28.65
![layer5_force_contact pt_a1](out\layer5_force_contact\pt_a1.png)

### [PASS] layer5_force_contact         pt_a2
- **imu_count**: 1739
- **raw_min**: 0
- **raw_max**: 2324
- **raw_median**: 0
- **up_transitions**: 2
- **down_transitions**: 2
- **contact_episodes**: 2
- **mean_episode_samples**: 319
- **min_episode_samples**: 186
- **glitch_count**: 0
- **contact_duty_pct**: 36.69
![layer5_force_contact pt_a2](out\layer5_force_contact\pt_a2.png)

### [PASS] layer5_force_contact         pt_circle1
- **imu_count**: 2221
- **raw_min**: 0
- **raw_max**: 2275
- **raw_median**: 0
- **up_transitions**: 3
- **down_transitions**: 3
- **contact_episodes**: 3
- **mean_episode_samples**: 344.7
- **min_episode_samples**: 210
- **glitch_count**: 0
- **contact_duty_pct**: 46.56
![layer5_force_contact pt_circle1](out\layer5_force_contact\pt_circle1.png)

### [PASS] layer5_force_contact         pt_circle2
- **imu_count**: 2136
- **raw_min**: 0
- **raw_max**: 2378
- **raw_median**: 0
- **up_transitions**: 2
- **down_transitions**: 2
- **contact_episodes**: 2
- **mean_episode_samples**: 533
- **min_episode_samples**: 492
- **glitch_count**: 0
- **contact_duty_pct**: 49.91
![layer5_force_contact pt_circle2](out\layer5_force_contact\pt_circle2.png)

### [PASS] layer5_force_contact         pt_square1
- **imu_count**: 2785
- **raw_min**: 0
- **raw_max**: 2303
- **raw_median**: 0
- **up_transitions**: 4
- **down_transitions**: 4
- **contact_episodes**: 4
- **mean_episode_samples**: 321.8
- **min_episode_samples**: 270
- **glitch_count**: 0
- **contact_duty_pct**: 46.21
![layer5_force_contact pt_square1](out\layer5_force_contact\pt_square1.png)

### [PASS] layer5_force_contact         pt_square2
- **imu_count**: 2604
- **raw_min**: 0
- **raw_max**: 2343
- **raw_median**: 0
- **up_transitions**: 4
- **down_transitions**: 4
- **contact_episodes**: 4
- **mean_episode_samples**: 313.5
- **min_episode_samples**: 291
- **glitch_count**: 0
- **contact_duty_pct**: 48.16
![layer5_force_contact pt_square2](out\layer5_force_contact\pt_square2.png)

### [PASS] layer5_force_contact         pt_triangle1
- **imu_count**: 2443
- **raw_min**: 0
- **raw_max**: 2264
- **raw_median**: 0
- **up_transitions**: 3
- **down_transitions**: 3
- **contact_episodes**: 3
- **mean_episode_samples**: 333.3
- **min_episode_samples**: 250
- **glitch_count**: 0
- **contact_duty_pct**: 40.93
![layer5_force_contact pt_triangle1](out\layer5_force_contact\pt_triangle1.png)

### [PASS] layer5_force_contact         pt_triangle2
- **imu_count**: 2351
- **raw_min**: 0
- **raw_max**: 2357
- **raw_median**: 0
- **up_transitions**: 3
- **down_transitions**: 3
- **contact_episodes**: 3
- **mean_episode_samples**: 333.3
- **min_episode_samples**: 294
- **glitch_count**: 0
- **contact_duty_pct**: 42.54
![layer5_force_contact pt_triangle2](out\layer5_force_contact\pt_triangle2.png)

### [PASS] layer5_force_contact         rt_circle
- **imu_count**: 3279
- **raw_min**: 0
- **raw_max**: 2279
- **raw_median**: 0
- **up_transitions**: 2
- **down_transitions**: 2
- **contact_episodes**: 2
- **mean_episode_samples**: 751.5
- **min_episode_samples**: 81
- **glitch_count**: 0
- **contact_duty_pct**: 45.84
![layer5_force_contact rt_circle](out\layer5_force_contact\rt_circle.png)

### [PASS] layer5_force_contact         rt_square
- **imu_count**: 3291
- **raw_min**: 0
- **raw_max**: 2299
- **raw_median**: 0
- **up_transitions**: 1
- **down_transitions**: 1
- **contact_episodes**: 1
- **mean_episode_samples**: 1442
- **min_episode_samples**: 1442
- **glitch_count**: 0
- **contact_duty_pct**: 43.82
![layer5_force_contact rt_square](out\layer5_force_contact\rt_square.png)

### [PASS] layer5_force_contact         rt_triangle
- **imu_count**: 3108
- **raw_min**: 0
- **raw_max**: 2324
- **raw_median**: 0
- **up_transitions**: 2
- **down_transitions**: 2
- **contact_episodes**: 2
- **mean_episode_samples**: 638
- **min_episode_samples**: 277
- **glitch_count**: 0
- **contact_duty_pct**: 41.06
![layer5_force_contact rt_triangle](out\layer5_force_contact\rt_triangle.png)

### [PASS] layer6_ekf_end_to_end        rt_circle
- **uwb_updates**: 951
- **ekf_samples**: 947
- **cold_start_s**: 0
- **gate_reject_pct**: 0
- **accepted_total**: 3788
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04576
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.09241
- **stationary_speed_mean**: 0.05577
- **bias_final_m_s2**: 0.09773
- **bias_delta_last_2s**: 0.006541
- **pen_tip_rms_vs_irls_m**: 0.04975
- **mean_omega_rad_s**: 0.2945
- **max_omega_rad_s**: 2.331
- **omega_above_thresh_frac**: 0.01901
- **mean_anchor_residual_after_feedback_m**: 0.0255
![layer6_ekf_end_to_end rt_circle](out\layer6_ekf_end_to_end\rt_circle.png)

### [PASS] layer6_ekf_end_to_end        rt_square
- **uwb_updates**: 937
- **ekf_samples**: 933
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 3732
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04972
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.0913
- **stationary_speed_mean**: 0.199
- **bias_final_m_s2**: 0.02968
- **bias_delta_last_2s**: 0.002371
- **pen_tip_rms_vs_irls_m**: 0.05007
- **mean_omega_rad_s**: 0.2953
- **max_omega_rad_s**: 2.007
- **omega_above_thresh_frac**: 0.01393
- **mean_anchor_residual_after_feedback_m**: 0.02807
![layer6_ekf_end_to_end rt_square](out\layer6_ekf_end_to_end\rt_square.png)

### [FAIL] layer6_ekf_end_to_end        rt_triangle
- **uwb_updates**: 892
- **ekf_samples**: 888
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 3552
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.05378
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.08853
- **stationary_speed_mean**: 0.05215
- **bias_final_m_s2**: 0.05551
- **bias_delta_last_2s**: 0.001912
- **pen_tip_rms_vs_irls_m**: 0.05288
- **mean_omega_rad_s**: 0.3095
- **max_omega_rad_s**: 2.176
- **omega_above_thresh_frac**: 0.01914
- **mean_anchor_residual_after_feedback_m**: 0.02838
![layer6_ekf_end_to_end rt_triangle](out\layer6_ekf_end_to_end\rt_triangle.png)

### [PASS] layer6_ekf_end_to_end        middle-
- **uwb_updates**: 833
- **ekf_samples**: 829
- **cold_start_s**: 0.24
- **gate_reject_pct**: 0
- **accepted_total**: 3316
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.02672
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.007416
- **stationary_speed_mean**: 0.004956
- **bias_final_m_s2**: 0.0292
- **bias_delta_last_2s**: 0.002385
- **pen_tip_rms_vs_irls_m**: 0.03653
- **mean_omega_rad_s**: 0.06086
- **max_omega_rad_s**: 0.7736
- **omega_above_thresh_frac**: 0
- **mean_anchor_residual_after_feedback_m**: 0.01813
![layer6_ekf_end_to_end middle-](out\layer6_ekf_end_to_end\middle-.png)

### [FAIL] layer6_ekf_end_to_end        middleCCW_rot-
- **uwb_updates**: 1315
- **ekf_samples**: 1311
- **cold_start_s**: 0.08
- **gate_reject_pct**: 5.13
- **accepted_total**: 4975
- **rejected_total**: 269
- **rms_ekf_vs_irls_m**: 0.1475
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.02115
- **bias_final_m_s2**: 0.06495
- **bias_delta_last_2s**: 0.007255
- **pen_tip_rms_vs_irls_m**: 0.1477
- **mean_omega_rad_s**: 0.3563
- **max_omega_rad_s**: 6.438
- **omega_above_thresh_frac**: 0.06865
- **mean_anchor_residual_after_feedback_m**: 0.02291
![layer6_ekf_end_to_end middleCCW_rot-](out\layer6_ekf_end_to_end\middleCCW_rot-.png)

### [FAIL] layer6_ekf_end_to_end        middleCW_rot-
- **uwb_updates**: 1434
- **ekf_samples**: 1430
- **cold_start_s**: 0.141
- **gate_reject_pct**: 5.245
- **accepted_total**: 5420
- **rejected_total**: 300
- **rms_ekf_vs_irls_m**: 0.06675
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01341
- **stationary_speed_mean**: 0.006769
- **bias_final_m_s2**: 0.01413
- **bias_delta_last_2s**: 0.0007663
- **pen_tip_rms_vs_irls_m**: 0.08667
- **mean_omega_rad_s**: 0.3342
- **max_omega_rad_s**: 7.002
- **omega_above_thresh_frac**: 0.03986
- **mean_anchor_residual_after_feedback_m**: 0.02829
![layer6_ekf_end_to_end middleCW_rot-](out\layer6_ekf_end_to_end\middleCW_rot-.png)

### [PASS] layer6_ekf_end_to_end        ct_hline1
- **uwb_updates**: 429
- **ekf_samples**: 425
- **cold_start_s**: 0
- **gate_reject_pct**: 0
- **accepted_total**: 1700
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04437
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01076
- **stationary_speed_mean**: 0.08288
- **bias_final_m_s2**: 0.06942
- **bias_delta_last_2s**: 0.00227
- **pen_tip_rms_vs_irls_m**: 0.04701
- **mean_omega_rad_s**: 0.2576
- **max_omega_rad_s**: 1.851
- **omega_above_thresh_frac**: 0.01412
- **mean_anchor_residual_after_feedback_m**: 0.0214
![layer6_ekf_end_to_end ct_hline1](out\layer6_ekf_end_to_end\ct_hline1.png)

### [PASS] layer6_ekf_end_to_end        ct_hline2
- **uwb_updates**: 435
- **ekf_samples**: 431
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 1724
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03941
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01017
- **bias_final_m_s2**: 0.01671
- **bias_delta_last_2s**: 0.003107
- **pen_tip_rms_vs_irls_m**: 0.04054
- **mean_omega_rad_s**: 0.2055
- **max_omega_rad_s**: 1.364
- **omega_above_thresh_frac**: 0.009281
- **mean_anchor_residual_after_feedback_m**: 0.0203
![layer6_ekf_end_to_end ct_hline2](out\layer6_ekf_end_to_end\ct_hline2.png)

### [FAIL] layer6_ekf_end_to_end        ct_hline3
- **uwb_updates**: 438
- **ekf_samples**: 434
- **cold_start_s**: 0.52
- **gate_reject_pct**: 0
- **accepted_total**: 1736
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04027
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01132
- **stationary_speed_mean**: 0.001463
- **bias_final_m_s2**: 0.06351
- **bias_delta_last_2s**: 0.005826
- **pen_tip_rms_vs_irls_m**: 0.04508
- **mean_omega_rad_s**: 0.1887
- **max_omega_rad_s**: 3.052
- **omega_above_thresh_frac**: 0.01613
- **mean_anchor_residual_after_feedback_m**: 0.02132
![layer6_ekf_end_to_end ct_hline3](out\layer6_ekf_end_to_end\ct_hline3.png)

### [PASS] layer6_ekf_end_to_end        ct_vline1
- **uwb_updates**: 366
- **ekf_samples**: 362
- **cold_start_s**: 0.07999
- **gate_reject_pct**: 0
- **accepted_total**: 1448
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03233
- **latency_proxy_samples**: 6
- **latency_proxy_s**: 0.12
- **stationary_speed_mean_contact**: 0.01226
- **stationary_speed_mean**: 0.2628
- **bias_final_m_s2**: 0.02573
- **bias_delta_last_2s**: 0.00411
- **pen_tip_rms_vs_irls_m**: 0.04007
- **mean_omega_rad_s**: 0.198
- **max_omega_rad_s**: 1.117
- **omega_above_thresh_frac**: 0.002762
- **mean_anchor_residual_after_feedback_m**: 0.0215
![layer6_ekf_end_to_end ct_vline1](out\layer6_ekf_end_to_end\ct_vline1.png)

### [PASS] layer6_ekf_end_to_end        ct_vline2
- **uwb_updates**: 450
- **ekf_samples**: 446
- **cold_start_s**: 0.081
- **gate_reject_pct**: 0
- **accepted_total**: 1784
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0346
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01827
- **bias_final_m_s2**: 0.09712
- **bias_delta_last_2s**: 0.007213
- **pen_tip_rms_vs_irls_m**: 0.04324
- **mean_omega_rad_s**: 0.1757
- **max_omega_rad_s**: 2.919
- **omega_above_thresh_frac**: 0.008969
- **mean_anchor_residual_after_feedback_m**: 0.01901
![layer6_ekf_end_to_end ct_vline2](out\layer6_ekf_end_to_end\ct_vline2.png)

### [FAIL] layer6_ekf_end_to_end        ct_vline3
- **uwb_updates**: 467
- **ekf_samples**: 463
- **cold_start_s**: 0.079
- **gate_reject_pct**: 0
- **accepted_total**: 1852
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03126
- **latency_proxy_samples**: 12
- **latency_proxy_s**: 0.24
- **stationary_speed_mean_contact**: 0.01042
- **stationary_speed_mean**: 0.01799
- **bias_final_m_s2**: 0.01389
- **bias_delta_last_2s**: 0.00434
- **pen_tip_rms_vs_irls_m**: 0.04279
- **mean_omega_rad_s**: 0.1822
- **max_omega_rad_s**: 1.554
- **omega_above_thresh_frac**: 0.0108
- **mean_anchor_residual_after_feedback_m**: 0.02045
![layer6_ekf_end_to_end ct_vline3](out\layer6_ekf_end_to_end\ct_vline3.png)

### [PASS] layer6_ekf_end_to_end        ct_diagonal1
- **uwb_updates**: 514
- **ekf_samples**: 510
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2040
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04548
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01246
- **stationary_speed_mean**: 0.01182
- **bias_final_m_s2**: 0.03784
- **bias_delta_last_2s**: 0.002652
- **pen_tip_rms_vs_irls_m**: 0.05036
- **mean_omega_rad_s**: 0.1826
- **max_omega_rad_s**: 1.621
- **omega_above_thresh_frac**: 0.009804
- **mean_anchor_residual_after_feedback_m**: 0.02136
![layer6_ekf_end_to_end ct_diagonal1](out\layer6_ekf_end_to_end\ct_diagonal1.png)

### [FAIL] layer6_ekf_end_to_end        ct_diagonal2
- **uwb_updates**: 436
- **ekf_samples**: 432
- **cold_start_s**: 0.14
- **gate_reject_pct**: 0
- **accepted_total**: 1728
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.05012
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.022
- **bias_final_m_s2**: 0.05091
- **bias_delta_last_2s**: 0.01764
- **pen_tip_rms_vs_irls_m**: 0.05435
- **mean_omega_rad_s**: 0.2114
- **max_omega_rad_s**: 1.008
- **omega_above_thresh_frac**: 0.002315
- **mean_anchor_residual_after_feedback_m**: 0.02262
![layer6_ekf_end_to_end ct_diagonal2](out\layer6_ekf_end_to_end\ct_diagonal2.png)

### [FAIL] layer6_ekf_end_to_end        ct_diagonal3
- **uwb_updates**: 497
- **ekf_samples**: 493
- **cold_start_s**: 0.14
- **gate_reject_pct**: 0
- **accepted_total**: 1972
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.05467
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01174
- **stationary_speed_mean**: 0.366
- **bias_final_m_s2**: 0.04246
- **bias_delta_last_2s**: 0.004343
- **pen_tip_rms_vs_irls_m**: 0.05853
- **mean_omega_rad_s**: 0.23
- **max_omega_rad_s**: 3.065
- **omega_above_thresh_frac**: 0.01826
- **mean_anchor_residual_after_feedback_m**: 0.02136
![layer6_ekf_end_to_end ct_diagonal3](out\layer6_ekf_end_to_end\ct_diagonal3.png)

### [PASS] layer6_ekf_end_to_end        ct_circle1
- **uwb_updates**: 516
- **ekf_samples**: 512
- **cold_start_s**: 0.141
- **gate_reject_pct**: 0
- **accepted_total**: 2048
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03679
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01012
- **stationary_speed_mean**: 0
- **bias_final_m_s2**: 0.03584
- **bias_delta_last_2s**: 0.001457
- **pen_tip_rms_vs_irls_m**: 0.04053
- **mean_omega_rad_s**: 0.234
- **max_omega_rad_s**: 1.452
- **omega_above_thresh_frac**: 0.02344
- **mean_anchor_residual_after_feedback_m**: 0.02171
![layer6_ekf_end_to_end ct_circle1](out\layer6_ekf_end_to_end\ct_circle1.png)

### [PASS] layer6_ekf_end_to_end        ct_circle2
- **uwb_updates**: 609
- **ekf_samples**: 605
- **cold_start_s**: 0.08101
- **gate_reject_pct**: 0
- **accepted_total**: 2420
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03439
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01559
- **bias_final_m_s2**: 0.04296
- **bias_delta_last_2s**: 0.001029
- **pen_tip_rms_vs_irls_m**: 0.03687
- **mean_omega_rad_s**: 0.2106
- **max_omega_rad_s**: 1.412
- **omega_above_thresh_frac**: 0.008264
- **mean_anchor_residual_after_feedback_m**: 0.02046
![layer6_ekf_end_to_end ct_circle2](out\layer6_ekf_end_to_end\ct_circle2.png)

### [PASS] layer6_ekf_end_to_end        ct_circle3
- **uwb_updates**: 579
- **ekf_samples**: 575
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2300
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03658
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.006858
- **stationary_speed_mean**: 0.002462
- **bias_final_m_s2**: 0.02673
- **bias_delta_last_2s**: 0.002722
- **pen_tip_rms_vs_irls_m**: 0.0365
- **mean_omega_rad_s**: 0.2257
- **max_omega_rad_s**: 2.237
- **omega_above_thresh_frac**: 0.008696
- **mean_anchor_residual_after_feedback_m**: 0.02132
![layer6_ekf_end_to_end ct_circle3](out\layer6_ekf_end_to_end\ct_circle3.png)

### [PASS] layer6_ekf_end_to_end        ct_square1
- **uwb_updates**: 635
- **ekf_samples**: 631
- **cold_start_s**: 0.119
- **gate_reject_pct**: 0
- **accepted_total**: 2524
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03748
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.008733
- **stationary_speed_mean**: 0.02868
- **bias_final_m_s2**: 0.01464
- **bias_delta_last_2s**: 0.01808
- **pen_tip_rms_vs_irls_m**: 0.03828
- **mean_omega_rad_s**: 0.2147
- **max_omega_rad_s**: 1.019
- **omega_above_thresh_frac**: 0.001585
- **mean_anchor_residual_after_feedback_m**: 0.02191
![layer6_ekf_end_to_end ct_square1](out\layer6_ekf_end_to_end\ct_square1.png)

### [PASS] layer6_ekf_end_to_end        ct_square2
- **uwb_updates**: 677
- **ekf_samples**: 673
- **cold_start_s**: 0.07898
- **gate_reject_pct**: 0
- **accepted_total**: 2692
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03514
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.006438
- **stationary_speed_mean**: 0.003184
- **bias_final_m_s2**: 0.01512
- **bias_delta_last_2s**: 0.01939
- **pen_tip_rms_vs_irls_m**: 0.03639
- **mean_omega_rad_s**: 0.2086
- **max_omega_rad_s**: 1.484
- **omega_above_thresh_frac**: 0.0104
- **mean_anchor_residual_after_feedback_m**: 0.02092
![layer6_ekf_end_to_end ct_square2](out\layer6_ekf_end_to_end\ct_square2.png)

### [PASS] layer6_ekf_end_to_end        ct_square3
- **uwb_updates**: 666
- **ekf_samples**: 662
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2648
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03641
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.004984
- **stationary_speed_mean**: 0.007849
- **bias_final_m_s2**: 0.01889
- **bias_delta_last_2s**: 0.002218
- **pen_tip_rms_vs_irls_m**: 0.03743
- **mean_omega_rad_s**: 0.2258
- **max_omega_rad_s**: 2.661
- **omega_above_thresh_frac**: 0.01964
- **mean_anchor_residual_after_feedback_m**: 0.0216
![layer6_ekf_end_to_end ct_square3](out\layer6_ekf_end_to_end\ct_square3.png)

### [PASS] layer6_ekf_end_to_end        ct_triangle1
- **uwb_updates**: 557
- **ekf_samples**: 553
- **cold_start_s**: 0.08099
- **gate_reject_pct**: 0
- **accepted_total**: 2212
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04001
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01149
- **stationary_speed_mean**: 0.008243
- **bias_final_m_s2**: 0.02074
- **bias_delta_last_2s**: 0.003707
- **pen_tip_rms_vs_irls_m**: 0.04165
- **mean_omega_rad_s**: 0.2358
- **max_omega_rad_s**: 1.475
- **omega_above_thresh_frac**: 0.007233
- **mean_anchor_residual_after_feedback_m**: 0.02208
![layer6_ekf_end_to_end ct_triangle1](out\layer6_ekf_end_to_end\ct_triangle1.png)

### [PASS] layer6_ekf_end_to_end        ct_triangle2
- **uwb_updates**: 598
- **ekf_samples**: 594
- **cold_start_s**: 0.081
- **gate_reject_pct**: 0
- **accepted_total**: 2376
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0351
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.007828
- **bias_final_m_s2**: 0.01628
- **bias_delta_last_2s**: 0.0009483
- **pen_tip_rms_vs_irls_m**: 0.03509
- **mean_omega_rad_s**: 0.2196
- **max_omega_rad_s**: 2.483
- **omega_above_thresh_frac**: 0.01515
- **mean_anchor_residual_after_feedback_m**: 0.02091
![layer6_ekf_end_to_end ct_triangle2](out\layer6_ekf_end_to_end\ct_triangle2.png)

### [PASS] layer6_ekf_end_to_end        ct_triangle3
- **uwb_updates**: 519
- **ekf_samples**: 515
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2060
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04274
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.007357
- **stationary_speed_mean**: 0.003507
- **bias_final_m_s2**: 0.0596
- **bias_delta_last_2s**: 0.001043
- **pen_tip_rms_vs_irls_m**: 0.04178
- **mean_omega_rad_s**: 0.2602
- **max_omega_rad_s**: 1.546
- **omega_above_thresh_frac**: 0.02718
- **mean_anchor_residual_after_feedback_m**: 0.02375
![layer6_ekf_end_to_end ct_triangle3](out\layer6_ekf_end_to_end\ct_triangle3.png)

### [PASS] layer6_ekf_end_to_end        pt_circle1
- **uwb_updates**: 621
- **ekf_samples**: 617
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2468
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03763
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01317
- **bias_final_m_s2**: 0.009148
- **bias_delta_last_2s**: 0.006237
- **pen_tip_rms_vs_irls_m**: 0.04277
- **mean_omega_rad_s**: 0.2618
- **max_omega_rad_s**: 1.521
- **omega_above_thresh_frac**: 0.01135
- **mean_anchor_residual_after_feedback_m**: 0.02144
![layer6_ekf_end_to_end pt_circle1](out\layer6_ekf_end_to_end\pt_circle1.png)

### [PASS] layer6_ekf_end_to_end        pt_circle2
- **uwb_updates**: 594
- **ekf_samples**: 590
- **cold_start_s**: 0.221
- **gate_reject_pct**: 0
- **accepted_total**: 2360
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04064
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.02028
- **bias_final_m_s2**: 0.02325
- **bias_delta_last_2s**: 0.0004413
- **pen_tip_rms_vs_irls_m**: 0.0417
- **mean_omega_rad_s**: 0.3128
- **max_omega_rad_s**: 1.919
- **omega_above_thresh_frac**: 0.02881
- **mean_anchor_residual_after_feedback_m**: 0.02268
![layer6_ekf_end_to_end pt_circle2](out\layer6_ekf_end_to_end\pt_circle2.png)

### [PASS] layer6_ekf_end_to_end        pt_square1
- **uwb_updates**: 832
- **ekf_samples**: 828
- **cold_start_s**: 0
- **gate_reject_pct**: 0
- **accepted_total**: 3312
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03964
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01732
- **stationary_speed_mean**: 0.01023
- **bias_final_m_s2**: 0.0217
- **bias_delta_last_2s**: 0.002548
- **pen_tip_rms_vs_irls_m**: 0.04444
- **mean_omega_rad_s**: 0.2485
- **max_omega_rad_s**: 1.946
- **omega_above_thresh_frac**: 0.02536
- **mean_anchor_residual_after_feedback_m**: 0.02169
![layer6_ekf_end_to_end pt_square1](out\layer6_ekf_end_to_end\pt_square1.png)

### [PASS] layer6_ekf_end_to_end        pt_square2
- **uwb_updates**: 732
- **ekf_samples**: 728
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2912
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04397
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.02745
- **stationary_speed_mean**: 0.005647
- **bias_final_m_s2**: 0.01835
- **bias_delta_last_2s**: 0.00415
- **pen_tip_rms_vs_irls_m**: 0.04779
- **mean_omega_rad_s**: 0.2917
- **max_omega_rad_s**: 1.993
- **omega_above_thresh_frac**: 0.03709
- **mean_anchor_residual_after_feedback_m**: 0.02384
![layer6_ekf_end_to_end pt_square2](out\layer6_ekf_end_to_end\pt_square2.png)

### [PASS] layer6_ekf_end_to_end        pt_triangle1
- **uwb_updates**: 673
- **ekf_samples**: 669
- **cold_start_s**: 0.1
- **gate_reject_pct**: 0
- **accepted_total**: 2676
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04213
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01246
- **stationary_speed_mean**: 0.01608
- **bias_final_m_s2**: 0.05745
- **bias_delta_last_2s**: 0.001094
- **pen_tip_rms_vs_irls_m**: 0.04363
- **mean_omega_rad_s**: 0.2274
- **max_omega_rad_s**: 1.68
- **omega_above_thresh_frac**: 0.008969
- **mean_anchor_residual_after_feedback_m**: 0.02234
![layer6_ekf_end_to_end pt_triangle1](out\layer6_ekf_end_to_end\pt_triangle1.png)

### [PASS] layer6_ekf_end_to_end        pt_triangle2
- **uwb_updates**: 649
- **ekf_samples**: 645
- **cold_start_s**: 0.08099
- **gate_reject_pct**: 0
- **accepted_total**: 2580
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04274
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01775
- **bias_final_m_s2**: 0.002343
- **bias_delta_last_2s**: 0.0007536
- **pen_tip_rms_vs_irls_m**: 0.04589
- **mean_omega_rad_s**: 0.2371
- **max_omega_rad_s**: 2.211
- **omega_above_thresh_frac**: 0.003101
- **mean_anchor_residual_after_feedback_m**: 0.02287
![layer6_ekf_end_to_end pt_triangle2](out\layer6_ekf_end_to_end\pt_triangle2.png)

### [PASS] layer6_ekf_end_to_end        pt_a1
- **uwb_updates**: 523
- **ekf_samples**: 519
- **cold_start_s**: 0.079
- **gate_reject_pct**: 0
- **accepted_total**: 2076
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03409
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.01001
- **stationary_speed_mean**: 0.006293
- **bias_final_m_s2**: 0.05379
- **bias_delta_last_2s**: 0.005247
- **pen_tip_rms_vs_irls_m**: 0.03838
- **mean_omega_rad_s**: 0.2371
- **max_omega_rad_s**: 1.787
- **omega_above_thresh_frac**: 0.02505
- **mean_anchor_residual_after_feedback_m**: 0.01921
![layer6_ekf_end_to_end pt_a1](out\layer6_ekf_end_to_end\pt_a1.png)

### [FAIL] layer6_ekf_end_to_end        pt_a2
- **uwb_updates**: 488
- **ekf_samples**: 484
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 1936
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03436
- **latency_proxy_samples**: 12
- **latency_proxy_s**: 0.24
- **stationary_speed_mean_contact**: 0.01945
- **stationary_speed_mean**: 0.06422
- **bias_final_m_s2**: 0.04064
- **bias_delta_last_2s**: 0.006736
- **pen_tip_rms_vs_irls_m**: 0.03815
- **mean_omega_rad_s**: 0.2645
- **max_omega_rad_s**: 2.693
- **omega_above_thresh_frac**: 0.03926
- **mean_anchor_residual_after_feedback_m**: 0.02009
![layer6_ekf_end_to_end pt_a2](out\layer6_ekf_end_to_end\pt_a2.png)

### [PASS] layer6_ekf_end_to_end        hello_s1
- **uwb_updates**: 885
- **ekf_samples**: 881
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 3524
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04034
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.02
- **stationary_speed_mean_contact**: 0.03625
- **stationary_speed_mean**: 0.0717
- **bias_final_m_s2**: 0.02366
- **bias_delta_last_2s**: 0.001646
- **pen_tip_rms_vs_irls_m**: 0.04598
- **mean_omega_rad_s**: 0.3529
- **max_omega_rad_s**: 5.692
- **omega_above_thresh_frac**: 0.05902
- **mean_anchor_residual_after_feedback_m**: 0.0227
![layer6_ekf_end_to_end hello_s1](out\layer6_ekf_end_to_end\hello_s1.png)

### [PASS] layer6_ekf_end_to_end        hello_s2
- **uwb_updates**: 819
- **ekf_samples**: 815
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 3260
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0397
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.04716
- **stationary_speed_mean**: 0.2003
- **bias_final_m_s2**: 0.01866
- **bias_delta_last_2s**: 0.00244
- **pen_tip_rms_vs_irls_m**: 0.04231
- **mean_omega_rad_s**: 0.3447
- **max_omega_rad_s**: 3.365
- **omega_above_thresh_frac**: 0.05153
- **mean_anchor_residual_after_feedback_m**: 0.02362
![layer6_ekf_end_to_end hello_s2](out\layer6_ekf_end_to_end\hello_s2.png)

### [PASS] layer6_ekf_end_to_end        abc_s1
- **uwb_updates**: 910
- **ekf_samples**: 906
- **cold_start_s**: 0.07999
- **gate_reject_pct**: 0
- **accepted_total**: 3624
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03274
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.03218
- **stationary_speed_mean**: 0.1607
- **bias_final_m_s2**: 0.01653
- **bias_delta_last_2s**: 0.002644
- **pen_tip_rms_vs_irls_m**: 0.03758
- **mean_omega_rad_s**: 0.2865
- **max_omega_rad_s**: 3.326
- **omega_above_thresh_frac**: 0.04084
- **mean_anchor_residual_after_feedback_m**: 0.02064
![layer6_ekf_end_to_end abc_s1](out\layer6_ekf_end_to_end\abc_s1.png)

### [PASS] layer6_ekf_end_to_end        abc_s2
- **uwb_updates**: 789
- **ekf_samples**: 785
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 3140
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0418
- **latency_proxy_samples**: 3
- **latency_proxy_s**: 0.06
- **stationary_speed_mean_contact**: 0.04736
- **stationary_speed_mean**: 0.09878
- **bias_final_m_s2**: 0.05593
- **bias_delta_last_2s**: 0.001038
- **pen_tip_rms_vs_irls_m**: 0.0434
- **mean_omega_rad_s**: 0.2991
- **max_omega_rad_s**: 2.591
- **omega_above_thresh_frac**: 0.02803
- **mean_anchor_residual_after_feedback_m**: 0.0238
![layer6_ekf_end_to_end abc_s2](out\layer6_ekf_end_to_end\abc_s2.png)

### [PASS] layer6_ekf_end_to_end        ABC_b1
- **uwb_updates**: 1102
- **ekf_samples**: 1098
- **cold_start_s**: 0.07898
- **gate_reject_pct**: 0
- **accepted_total**: 4392
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04124
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.05344
- **stationary_speed_mean**: 0.1256
- **bias_final_m_s2**: 0.01519
- **bias_delta_last_2s**: 0.001533
- **pen_tip_rms_vs_irls_m**: 0.04505
- **mean_omega_rad_s**: 0.3177
- **max_omega_rad_s**: 2.724
- **omega_above_thresh_frac**: 0.04463
- **mean_anchor_residual_after_feedback_m**: 0.02421
![layer6_ekf_end_to_end ABC_b1](out\layer6_ekf_end_to_end\ABC_b1.png)

### [PASS] layer6_ekf_end_to_end        ABC_b2
- **uwb_updates**: 1115
- **ekf_samples**: 1111
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 4444
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04309
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.02
- **stationary_speed_mean_contact**: 0.05201
- **stationary_speed_mean**: 0.09997
- **bias_final_m_s2**: 0.04698
- **bias_delta_last_2s**: 0.0006022
- **pen_tip_rms_vs_irls_m**: 0.04585
- **mean_omega_rad_s**: 0.3262
- **max_omega_rad_s**: 3.618
- **omega_above_thresh_frac**: 0.05221
- **mean_anchor_residual_after_feedback_m**: 0.02445
![layer6_ekf_end_to_end ABC_b2](out\layer6_ekf_end_to_end\ABC_b2.png)

### [PASS] layer6_ekf_end_to_end        HELLO_b1
- **uwb_updates**: 1492
- **ekf_samples**: 1488
- **cold_start_s**: 0.07998
- **gate_reject_pct**: 0
- **accepted_total**: 5952
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04111
- **latency_proxy_samples**: 2
- **latency_proxy_s**: 0.04
- **stationary_speed_mean_contact**: 0.06258
- **stationary_speed_mean**: 0.07524
- **bias_final_m_s2**: 0.009381
- **bias_delta_last_2s**: 0.001943
- **pen_tip_rms_vs_irls_m**: 0.04769
- **mean_omega_rad_s**: 0.3637
- **max_omega_rad_s**: 5.608
- **omega_above_thresh_frac**: 0.05444
- **mean_anchor_residual_after_feedback_m**: 0.02311
![layer6_ekf_end_to_end HELLO_b1](out\layer6_ekf_end_to_end\HELLO_b1.png)

### [PASS] layer6_ekf_end_to_end        HELLO_b2
- **uwb_updates**: 1463
- **ekf_samples**: 1459
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 5836
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04221
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean_contact**: 0.07013
- **stationary_speed_mean**: 0.06118
- **bias_final_m_s2**: 0.03026
- **bias_delta_last_2s**: 0.000483
- **pen_tip_rms_vs_irls_m**: 0.0468
- **mean_omega_rad_s**: 0.3551
- **max_omega_rad_s**: 3.478
- **omega_above_thresh_frac**: 0.04798
- **mean_anchor_residual_after_feedback_m**: 0.02421
![layer6_ekf_end_to_end HELLO_b2](out\layer6_ekf_end_to_end\HELLO_b2.png)

### [PASS] layer6_ekf_end_to_end        ct_hline1+contact
- **uwb_updates**: 429
- **ekf_samples**: 425
- **cold_start_s**: 0
- **gate_reject_pct**: 0
- **accepted_total**: 1700
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04468
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.02627
- **bias_final_m_s2**: 0.0479
- **bias_delta_last_2s**: 0.000452
- **pen_tip_rms_vs_irls_m**: 0.04748
- **mean_omega_rad_s**: 0.2576
- **max_omega_rad_s**: 1.851
- **omega_above_thresh_frac**: 0.01412
- **mean_anchor_residual_after_feedback_m**: 0.0214
![layer6_ekf_end_to_end ct_hline1+contact](out\layer6_ekf_end_to_end\ct_hline1+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_hline2+contact
- **uwb_updates**: 435
- **ekf_samples**: 431
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 1724
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03956
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **bias_final_m_s2**: 0.02426
- **bias_delta_last_2s**: 0.002113
- **pen_tip_rms_vs_irls_m**: 0.03995
- **mean_omega_rad_s**: 0.2055
- **max_omega_rad_s**: 1.364
- **omega_above_thresh_frac**: 0.009281
- **mean_anchor_residual_after_feedback_m**: 0.0203
![layer6_ekf_end_to_end ct_hline2+contact](out\layer6_ekf_end_to_end\ct_hline2+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_hline3+contact
- **uwb_updates**: 438
- **ekf_samples**: 434
- **cold_start_s**: 0.52
- **gate_reject_pct**: 0
- **accepted_total**: 1736
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04006
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.002186
- **bias_final_m_s2**: 0.03631
- **bias_delta_last_2s**: 0.002886
- **pen_tip_rms_vs_irls_m**: 0.04621
- **mean_omega_rad_s**: 0.1887
- **max_omega_rad_s**: 3.052
- **omega_above_thresh_frac**: 0.01613
- **mean_anchor_residual_after_feedback_m**: 0.02132
![layer6_ekf_end_to_end ct_hline3+contact](out\layer6_ekf_end_to_end\ct_hline3+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_vline1+contact
- **uwb_updates**: 366
- **ekf_samples**: 362
- **cold_start_s**: 0.07999
- **gate_reject_pct**: 0
- **accepted_total**: 1448
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03285
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.2671
- **bias_final_m_s2**: 0.03056
- **bias_delta_last_2s**: 0.004811
- **pen_tip_rms_vs_irls_m**: 0.04095
- **mean_omega_rad_s**: 0.198
- **max_omega_rad_s**: 1.117
- **omega_above_thresh_frac**: 0.002762
- **mean_anchor_residual_after_feedback_m**: 0.0215
![layer6_ekf_end_to_end ct_vline1+contact](out\layer6_ekf_end_to_end\ct_vline1+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_vline2+contact
- **uwb_updates**: 450
- **ekf_samples**: 446
- **cold_start_s**: 0.081
- **gate_reject_pct**: 0
- **accepted_total**: 1784
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03433
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **bias_final_m_s2**: 0.05747
- **bias_delta_last_2s**: 0.006579
- **pen_tip_rms_vs_irls_m**: 0.04351
- **mean_omega_rad_s**: 0.1757
- **max_omega_rad_s**: 2.919
- **omega_above_thresh_frac**: 0.008969
- **mean_anchor_residual_after_feedback_m**: 0.01901
![layer6_ekf_end_to_end ct_vline2+contact](out\layer6_ekf_end_to_end\ct_vline2+contact.png)

### [FAIL] layer6_ekf_end_to_end        ct_vline3+contact
- **uwb_updates**: 467
- **ekf_samples**: 463
- **cold_start_s**: 0.079
- **gate_reject_pct**: 0
- **accepted_total**: 1852
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03151
- **latency_proxy_samples**: 14
- **latency_proxy_s**: 0.28
- **stationary_speed_mean**: 0.0164
- **bias_final_m_s2**: 0.01021
- **bias_delta_last_2s**: 0.005762
- **pen_tip_rms_vs_irls_m**: 0.04161
- **mean_omega_rad_s**: 0.1822
- **max_omega_rad_s**: 1.554
- **omega_above_thresh_frac**: 0.0108
- **mean_anchor_residual_after_feedback_m**: 0.02045
![layer6_ekf_end_to_end ct_vline3+contact](out\layer6_ekf_end_to_end\ct_vline3+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_diagonal1+contact
- **uwb_updates**: 514
- **ekf_samples**: 510
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2040
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0453
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.01125
- **bias_final_m_s2**: 0.01659
- **bias_delta_last_2s**: 0.004638
- **pen_tip_rms_vs_irls_m**: 0.04931
- **mean_omega_rad_s**: 0.1826
- **max_omega_rad_s**: 1.621
- **omega_above_thresh_frac**: 0.009804
- **mean_anchor_residual_after_feedback_m**: 0.02136
![layer6_ekf_end_to_end ct_diagonal1+contact](out\layer6_ekf_end_to_end\ct_diagonal1+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_diagonal2+contact
- **uwb_updates**: 436
- **ekf_samples**: 432
- **cold_start_s**: 0.14
- **gate_reject_pct**: 0
- **accepted_total**: 1728
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.05061
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **bias_final_m_s2**: 0.0722
- **bias_delta_last_2s**: 0.02171
- **pen_tip_rms_vs_irls_m**: 0.05511
- **mean_omega_rad_s**: 0.2114
- **max_omega_rad_s**: 1.008
- **omega_above_thresh_frac**: 0.002315
- **mean_anchor_residual_after_feedback_m**: 0.02262
![layer6_ekf_end_to_end ct_diagonal2+contact](out\layer6_ekf_end_to_end\ct_diagonal2+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_diagonal3+contact
- **uwb_updates**: 497
- **ekf_samples**: 493
- **cold_start_s**: 0.14
- **gate_reject_pct**: 0
- **accepted_total**: 1972
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.05429
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.3754
- **bias_final_m_s2**: 0.03812
- **bias_delta_last_2s**: 0.0006766
- **pen_tip_rms_vs_irls_m**: 0.05795
- **mean_omega_rad_s**: 0.23
- **max_omega_rad_s**: 3.065
- **omega_above_thresh_frac**: 0.01826
- **mean_anchor_residual_after_feedback_m**: 0.02136
![layer6_ekf_end_to_end ct_diagonal3+contact](out\layer6_ekf_end_to_end\ct_diagonal3+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_circle1+contact
- **uwb_updates**: 516
- **ekf_samples**: 512
- **cold_start_s**: 0.141
- **gate_reject_pct**: 0
- **accepted_total**: 2048
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03637
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0
- **bias_final_m_s2**: 0.02476
- **bias_delta_last_2s**: 0.000755
- **pen_tip_rms_vs_irls_m**: 0.03947
- **mean_omega_rad_s**: 0.234
- **max_omega_rad_s**: 1.452
- **omega_above_thresh_frac**: 0.02344
- **mean_anchor_residual_after_feedback_m**: 0.02171
![layer6_ekf_end_to_end ct_circle1+contact](out\layer6_ekf_end_to_end\ct_circle1+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_circle2+contact
- **uwb_updates**: 609
- **ekf_samples**: 605
- **cold_start_s**: 0.08101
- **gate_reject_pct**: 0
- **accepted_total**: 2420
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03459
- **latency_proxy_s**: 0
- **bias_final_m_s2**: 0.02232
- **bias_delta_last_2s**: 0.0008306
- **pen_tip_rms_vs_irls_m**: 0.03649
- **mean_omega_rad_s**: 0.2106
- **max_omega_rad_s**: 1.412
- **omega_above_thresh_frac**: 0.008264
- **mean_anchor_residual_after_feedback_m**: 0.02046
![layer6_ekf_end_to_end ct_circle2+contact](out\layer6_ekf_end_to_end\ct_circle2+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_circle3+contact
- **uwb_updates**: 579
- **ekf_samples**: 575
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2300
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03642
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.004263
- **bias_final_m_s2**: 0.01411
- **bias_delta_last_2s**: 0.002571
- **pen_tip_rms_vs_irls_m**: 0.03651
- **mean_omega_rad_s**: 0.2257
- **max_omega_rad_s**: 2.237
- **omega_above_thresh_frac**: 0.008696
- **mean_anchor_residual_after_feedback_m**: 0.02132
![layer6_ekf_end_to_end ct_circle3+contact](out\layer6_ekf_end_to_end\ct_circle3+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_square1+contact
- **uwb_updates**: 635
- **ekf_samples**: 631
- **cold_start_s**: 0.119
- **gate_reject_pct**: 0
- **accepted_total**: 2524
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03658
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.002195
- **bias_final_m_s2**: 0.04525
- **bias_delta_last_2s**: 0.05079
- **pen_tip_rms_vs_irls_m**: 0.03716
- **mean_omega_rad_s**: 0.2147
- **max_omega_rad_s**: 1.019
- **omega_above_thresh_frac**: 0.001585
- **mean_anchor_residual_after_feedback_m**: 0.02191
![layer6_ekf_end_to_end ct_square1+contact](out\layer6_ekf_end_to_end\ct_square1+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_square2+contact
- **uwb_updates**: 677
- **ekf_samples**: 673
- **cold_start_s**: 0.07898
- **gate_reject_pct**: 0
- **accepted_total**: 2692
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03509
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.001083
- **bias_final_m_s2**: 0.05999
- **bias_delta_last_2s**: 0.01388
- **pen_tip_rms_vs_irls_m**: 0.03723
- **mean_omega_rad_s**: 0.2086
- **max_omega_rad_s**: 1.484
- **omega_above_thresh_frac**: 0.0104
- **mean_anchor_residual_after_feedback_m**: 0.02092
![layer6_ekf_end_to_end ct_square2+contact](out\layer6_ekf_end_to_end\ct_square2+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_square3+contact
- **uwb_updates**: 666
- **ekf_samples**: 662
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2648
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03667
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.005988
- **bias_final_m_s2**: 0.02021
- **bias_delta_last_2s**: 0.001014
- **pen_tip_rms_vs_irls_m**: 0.03766
- **mean_omega_rad_s**: 0.2258
- **max_omega_rad_s**: 2.661
- **omega_above_thresh_frac**: 0.01964
- **mean_anchor_residual_after_feedback_m**: 0.0216
![layer6_ekf_end_to_end ct_square3+contact](out\layer6_ekf_end_to_end\ct_square3+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_triangle1+contact
- **uwb_updates**: 557
- **ekf_samples**: 553
- **cold_start_s**: 0.08099
- **gate_reject_pct**: 0
- **accepted_total**: 2212
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03961
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.2412
- **bias_final_m_s2**: 0.006055
- **bias_delta_last_2s**: 0.002463
- **pen_tip_rms_vs_irls_m**: 0.04154
- **mean_omega_rad_s**: 0.2358
- **max_omega_rad_s**: 1.475
- **omega_above_thresh_frac**: 0.007233
- **mean_anchor_residual_after_feedback_m**: 0.02208
![layer6_ekf_end_to_end ct_triangle1+contact](out\layer6_ekf_end_to_end\ct_triangle1+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_triangle2+contact
- **uwb_updates**: 598
- **ekf_samples**: 594
- **cold_start_s**: 0.081
- **gate_reject_pct**: 0
- **accepted_total**: 2376
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03505
- **latency_proxy_s**: 0
- **bias_final_m_s2**: 0.02987
- **bias_delta_last_2s**: 0.004365
- **pen_tip_rms_vs_irls_m**: 0.03537
- **mean_omega_rad_s**: 0.2196
- **max_omega_rad_s**: 2.483
- **omega_above_thresh_frac**: 0.01515
- **mean_anchor_residual_after_feedback_m**: 0.02091
![layer6_ekf_end_to_end ct_triangle2+contact](out\layer6_ekf_end_to_end\ct_triangle2+contact.png)

### [PASS] layer6_ekf_end_to_end        ct_triangle3+contact
- **uwb_updates**: 519
- **ekf_samples**: 515
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2060
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0431
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.005116
- **bias_final_m_s2**: 0.05329
- **bias_delta_last_2s**: 0.0007534
- **pen_tip_rms_vs_irls_m**: 0.04268
- **mean_omega_rad_s**: 0.2602
- **max_omega_rad_s**: 1.546
- **omega_above_thresh_frac**: 0.02718
- **mean_anchor_residual_after_feedback_m**: 0.02375
![layer6_ekf_end_to_end ct_triangle3+contact](out\layer6_ekf_end_to_end\ct_triangle3+contact.png)

### [PASS] layer6_ekf_end_to_end        pt_circle1+contact
- **uwb_updates**: 621
- **ekf_samples**: 617
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2468
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0387
- **latency_proxy_s**: 0
- **bias_final_m_s2**: 0.09649
- **bias_delta_last_2s**: 0.008838
- **pen_tip_rms_vs_irls_m**: 0.04455
- **mean_omega_rad_s**: 0.2618
- **max_omega_rad_s**: 1.521
- **omega_above_thresh_frac**: 0.01135
- **mean_anchor_residual_after_feedback_m**: 0.02144
![layer6_ekf_end_to_end pt_circle1+contact](out\layer6_ekf_end_to_end\pt_circle1+contact.png)

### [PASS] layer6_ekf_end_to_end        pt_circle2+contact
- **uwb_updates**: 594
- **ekf_samples**: 590
- **cold_start_s**: 0.221
- **gate_reject_pct**: 0
- **accepted_total**: 2360
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04056
- **latency_proxy_s**: 0
- **bias_final_m_s2**: 0.05745
- **bias_delta_last_2s**: 0.005786
- **pen_tip_rms_vs_irls_m**: 0.04268
- **mean_omega_rad_s**: 0.3128
- **max_omega_rad_s**: 1.919
- **omega_above_thresh_frac**: 0.02881
- **mean_anchor_residual_after_feedback_m**: 0.02268
![layer6_ekf_end_to_end pt_circle2+contact](out\layer6_ekf_end_to_end\pt_circle2+contact.png)

### [PASS] layer6_ekf_end_to_end        pt_square1+contact
- **uwb_updates**: 832
- **ekf_samples**: 828
- **cold_start_s**: 0
- **gate_reject_pct**: 0
- **accepted_total**: 3312
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03969
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.006012
- **bias_final_m_s2**: 0.02992
- **bias_delta_last_2s**: 0.001634
- **pen_tip_rms_vs_irls_m**: 0.04481
- **mean_omega_rad_s**: 0.2485
- **max_omega_rad_s**: 1.946
- **omega_above_thresh_frac**: 0.02536
- **mean_anchor_residual_after_feedback_m**: 0.02169
![layer6_ekf_end_to_end pt_square1+contact](out\layer6_ekf_end_to_end\pt_square1+contact.png)

### [PASS] layer6_ekf_end_to_end        pt_square2+contact
- **uwb_updates**: 732
- **ekf_samples**: 728
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 2912
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04418
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.001427
- **bias_final_m_s2**: 0.008474
- **bias_delta_last_2s**: 0.004063
- **pen_tip_rms_vs_irls_m**: 0.04778
- **mean_omega_rad_s**: 0.2917
- **max_omega_rad_s**: 1.993
- **omega_above_thresh_frac**: 0.03709
- **mean_anchor_residual_after_feedback_m**: 0.02384
![layer6_ekf_end_to_end pt_square2+contact](out\layer6_ekf_end_to_end\pt_square2+contact.png)

### [PASS] layer6_ekf_end_to_end        pt_triangle1+contact
- **uwb_updates**: 673
- **ekf_samples**: 669
- **cold_start_s**: 0.1
- **gate_reject_pct**: 0
- **accepted_total**: 2676
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0423
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.02259
- **bias_final_m_s2**: 0.05014
- **bias_delta_last_2s**: 0.001745
- **pen_tip_rms_vs_irls_m**: 0.04357
- **mean_omega_rad_s**: 0.2274
- **max_omega_rad_s**: 1.68
- **omega_above_thresh_frac**: 0.008969
- **mean_anchor_residual_after_feedback_m**: 0.02234
![layer6_ekf_end_to_end pt_triangle1+contact](out\layer6_ekf_end_to_end\pt_triangle1+contact.png)

### [PASS] layer6_ekf_end_to_end        pt_triangle2+contact
- **uwb_updates**: 649
- **ekf_samples**: 645
- **cold_start_s**: 0.08099
- **gate_reject_pct**: 0
- **accepted_total**: 2580
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04205
- **latency_proxy_s**: 0
- **bias_final_m_s2**: 0.01529
- **bias_delta_last_2s**: 0.001427
- **pen_tip_rms_vs_irls_m**: 0.04501
- **mean_omega_rad_s**: 0.2371
- **max_omega_rad_s**: 2.211
- **omega_above_thresh_frac**: 0.003101
- **mean_anchor_residual_after_feedback_m**: 0.02287
![layer6_ekf_end_to_end pt_triangle2+contact](out\layer6_ekf_end_to_end\pt_triangle2+contact.png)

### [PASS] layer6_ekf_end_to_end        pt_a1+contact
- **uwb_updates**: 523
- **ekf_samples**: 519
- **cold_start_s**: 0.079
- **gate_reject_pct**: 0
- **accepted_total**: 2076
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03836
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.005755
- **bias_final_m_s2**: 0.05014
- **bias_delta_last_2s**: 0.0009341
- **pen_tip_rms_vs_irls_m**: 0.04184
- **mean_omega_rad_s**: 0.2371
- **max_omega_rad_s**: 1.787
- **omega_above_thresh_frac**: 0.02505
- **mean_anchor_residual_after_feedback_m**: 0.01921
![layer6_ekf_end_to_end pt_a1+contact](out\layer6_ekf_end_to_end\pt_a1+contact.png)

### [FAIL] layer6_ekf_end_to_end        pt_a2+contact
- **uwb_updates**: 488
- **ekf_samples**: 484
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 1936
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0351
- **latency_proxy_samples**: 13
- **latency_proxy_s**: 0.26
- **stationary_speed_mean**: 0.04076
- **bias_final_m_s2**: 0.05037
- **bias_delta_last_2s**: 0.006177
- **pen_tip_rms_vs_irls_m**: 0.03906
- **mean_omega_rad_s**: 0.2645
- **max_omega_rad_s**: 2.693
- **omega_above_thresh_frac**: 0.03926
- **mean_anchor_residual_after_feedback_m**: 0.02009
![layer6_ekf_end_to_end pt_a2+contact](out\layer6_ekf_end_to_end\pt_a2+contact.png)

### [PASS] layer6_ekf_end_to_end        hello_s1+contact
- **uwb_updates**: 885
- **ekf_samples**: 881
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 3524
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04202
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.02
- **stationary_speed_mean**: 0.1109
- **bias_final_m_s2**: 0.04213
- **bias_delta_last_2s**: 0.002638
- **pen_tip_rms_vs_irls_m**: 0.04863
- **mean_omega_rad_s**: 0.3529
- **max_omega_rad_s**: 5.692
- **omega_above_thresh_frac**: 0.05902
- **mean_anchor_residual_after_feedback_m**: 0.0227
![layer6_ekf_end_to_end hello_s1+contact](out\layer6_ekf_end_to_end\hello_s1+contact.png)

### [PASS] layer6_ekf_end_to_end        hello_s2+contact
- **uwb_updates**: 819
- **ekf_samples**: 815
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 3260
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04054
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.2012
- **bias_final_m_s2**: 0.01257
- **bias_delta_last_2s**: 0.001282
- **pen_tip_rms_vs_irls_m**: 0.04205
- **mean_omega_rad_s**: 0.3447
- **max_omega_rad_s**: 3.365
- **omega_above_thresh_frac**: 0.05153
- **mean_anchor_residual_after_feedback_m**: 0.02362
![layer6_ekf_end_to_end hello_s2+contact](out\layer6_ekf_end_to_end\hello_s2+contact.png)

### [PASS] layer6_ekf_end_to_end        abc_s1+contact
- **uwb_updates**: 910
- **ekf_samples**: 906
- **cold_start_s**: 0.07999
- **gate_reject_pct**: 0
- **accepted_total**: 3624
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.03261
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.1567
- **bias_final_m_s2**: 0.01143
- **bias_delta_last_2s**: 0.0005195
- **pen_tip_rms_vs_irls_m**: 0.03755
- **mean_omega_rad_s**: 0.2865
- **max_omega_rad_s**: 3.326
- **omega_above_thresh_frac**: 0.04084
- **mean_anchor_residual_after_feedback_m**: 0.02064
![layer6_ekf_end_to_end abc_s1+contact](out\layer6_ekf_end_to_end\abc_s1+contact.png)

### [PASS] layer6_ekf_end_to_end        abc_s2+contact
- **uwb_updates**: 789
- **ekf_samples**: 785
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 3140
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04257
- **latency_proxy_samples**: 3
- **latency_proxy_s**: 0.06
- **stationary_speed_mean**: 0.08417
- **bias_final_m_s2**: 0.02078
- **bias_delta_last_2s**: 0.001117
- **pen_tip_rms_vs_irls_m**: 0.04373
- **mean_omega_rad_s**: 0.2991
- **max_omega_rad_s**: 2.591
- **omega_above_thresh_frac**: 0.02803
- **mean_anchor_residual_after_feedback_m**: 0.0238
![layer6_ekf_end_to_end abc_s2+contact](out\layer6_ekf_end_to_end\abc_s2+contact.png)

### [FAIL] layer6_ekf_end_to_end        ABC_b1+contact
- **uwb_updates**: 1102
- **ekf_samples**: 1098
- **cold_start_s**: 0.07898
- **gate_reject_pct**: 3.597
- **accepted_total**: 4234
- **rejected_total**: 158
- **rms_ekf_vs_irls_m**: 0.1079
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.07533
- **bias_final_m_s2**: 0.01933
- **bias_delta_last_2s**: 0.0001594
- **pen_tip_rms_vs_irls_m**: 0.1075
- **mean_omega_rad_s**: 0.3177
- **max_omega_rad_s**: 2.724
- **omega_above_thresh_frac**: 0.04463
- **mean_anchor_residual_after_feedback_m**: 0.02421
![layer6_ekf_end_to_end ABC_b1+contact](out\layer6_ekf_end_to_end\ABC_b1+contact.png)

### [PASS] layer6_ekf_end_to_end        ABC_b2+contact
- **uwb_updates**: 1115
- **ekf_samples**: 1111
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 4444
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04447
- **latency_proxy_samples**: 1
- **latency_proxy_s**: 0.02
- **stationary_speed_mean**: 0.1023
- **bias_final_m_s2**: 0.02867
- **bias_delta_last_2s**: 0.001903
- **pen_tip_rms_vs_irls_m**: 0.04718
- **mean_omega_rad_s**: 0.3262
- **max_omega_rad_s**: 3.618
- **omega_above_thresh_frac**: 0.05221
- **mean_anchor_residual_after_feedback_m**: 0.02445
![layer6_ekf_end_to_end ABC_b2+contact](out\layer6_ekf_end_to_end\ABC_b2+contact.png)

### [PASS] layer6_ekf_end_to_end        HELLO_b1+contact
- **uwb_updates**: 1492
- **ekf_samples**: 1488
- **cold_start_s**: 0.07998
- **gate_reject_pct**: 0
- **accepted_total**: 5952
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.0411
- **latency_proxy_samples**: 2
- **latency_proxy_s**: 0.04
- **stationary_speed_mean**: 0.08363
- **bias_final_m_s2**: 0.01528
- **bias_delta_last_2s**: 0.001504
- **pen_tip_rms_vs_irls_m**: 0.04694
- **mean_omega_rad_s**: 0.3637
- **max_omega_rad_s**: 5.608
- **omega_above_thresh_frac**: 0.05444
- **mean_anchor_residual_after_feedback_m**: 0.02311
![layer6_ekf_end_to_end HELLO_b1+contact](out\layer6_ekf_end_to_end\HELLO_b1+contact.png)

### [PASS] layer6_ekf_end_to_end        HELLO_b2+contact
- **uwb_updates**: 1463
- **ekf_samples**: 1459
- **cold_start_s**: 0.08
- **gate_reject_pct**: 0
- **accepted_total**: 5836
- **rejected_total**: 0
- **rms_ekf_vs_irls_m**: 0.04262
- **latency_proxy_samples**: 0
- **latency_proxy_s**: 0
- **stationary_speed_mean**: 0.05972
- **bias_final_m_s2**: 0.03419
- **bias_delta_last_2s**: 0.0006402
- **pen_tip_rms_vs_irls_m**: 0.0469
- **mean_omega_rad_s**: 0.3551
- **max_omega_rad_s**: 3.478
- **omega_above_thresh_frac**: 0.04798
- **mean_anchor_residual_after_feedback_m**: 0.02421
![layer6_ekf_end_to_end HELLO_b2+contact](out\layer6_ekf_end_to_end\HELLO_b2+contact.png)
