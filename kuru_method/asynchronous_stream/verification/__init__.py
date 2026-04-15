"""
verification/ -- PolyCast async-stream verification layer.

A bottom-up, number-producing test battery that replays CSVs from
datasets_a3_ls/ and emits per-layer metrics + saved plots so the
first failing layer pinpoints the fault in the fusion pipeline.

Layers
------
    layer0_stream_health      -- packet loss, timing sanity
    layer1_uwb_raw            -- per-anchor raw + calibrated distance accuracy
    layer2_uwb_position       -- IRLS-only position (no IMU)
    layer3_imu_raw            -- quaternion stability, accel noise floor
    layer4_imu_integration    -- heading lock, body->wb transform, tag_z
    layer5_force_contact      -- FSR hysteresis / debounce health
    layer6_ekf_end_to_end     -- EKF tracking error, latency, gate rate

Run everything via  `python verification/run_all.py`.
"""
