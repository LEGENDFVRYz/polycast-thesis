"""
benchmark/run_test1_classroom.py — Classroom Simulation Test (Step 2)

Run this script IN THE SAME PROCESS as the Flask server by setting BENCHMARK=1
before starting main.py.  The BenchmarkLogger singleton is shared across all
threads in the same process, so latency hooks in _prototype_thread.py write to
the same session directory as the resource sampler below.

Usage (Raspberry Pi):
    Terminal 1:  BENCHMARK=1 python main.py
    Terminal 2:  BENCHMARK=1 python -m benchmark.run_test1_classroom --duration 3600

For a single-terminal workflow (60-minute run):
    BENCHMARK=1 python -m benchmark.run_test1_classroom --duration 3600
    (the script sets BENCHMARK before importing the server, so both share the same logger)

Note: Run the Flask+serial server FIRST so the logger singleton is already
constructed before this script calls get_logger().  If running both from the
same process entry point, ensure BENCHMARK=1 is set before any imports.
"""

import argparse
import os
import sys
import threading
import time

import psutil


def _resource_sampler(logger, stop_event: threading.Event, interval_s: float):
    """Background daemon: samples CPU/RAM and calls logger every interval_s."""
    while not stop_event.wait(timeout=interval_s):
        cpu = psutil.cpu_percent(interval=None)
        ram = psutil.virtual_memory().used / 1e6
        logger.on_resource_sample(cpu_percent=cpu, ram_mb=ram)


def run_classroom_test(duration_s: int = 3600, sample_interval: int = 5):
    os.environ["BENCHMARK"] = "1"

    from benchmark import get_logger
    logger = get_logger(extra_tags={"test": "classroom", "duration_s": duration_s})

    stop_event = threading.Event()
    sampler = threading.Thread(
        target=_resource_sampler,
        args=(logger, stop_event, float(sample_interval)),
        daemon=True,
        name="bm-resource-sampler",
    )
    sampler.start()

    total_min = duration_s // 60
    print(f"[TEST1] Classroom simulation — {total_min} min ({duration_s}s)")
    print(f"[TEST1] Session ID : {logger.session_id}")
    print(f"[TEST1] Output dir : {logger.session_dir}")
    print(f"[TEST1] Resource sampling every {sample_interval}s")
    print("[TEST1] Press Ctrl+C to stop early.\n")

    _print_phase_plan(duration_s)

    start_wall = time.time()
    try:
        for elapsed in range(duration_s):
            time.sleep(1)
            mins = elapsed // 60
            secs = elapsed % 60
            if secs == 0 and elapsed > 0:
                print(f"[TEST1] {mins:>3}m elapsed — running...")
    except KeyboardInterrupt:
        actual = round(time.time() - start_wall, 1)
        print(f"\n[TEST1] Stopped early at {actual}s.")

    stop_event.set()
    logger.stop()

    actual_s = round(time.time() - start_wall, 1)
    print(f"\n[TEST1] Done. Duration: {actual_s}s")
    print(f"[TEST1] CSVs saved to: {logger.session_dir}")


def _print_phase_plan(duration_s: int):
    """Print the classroom simulation phase schedule."""
    phases = [
        (0,  10, "Normal writing — baseline classroom behaviour"),
        (10, 20, "Continuous writing — sustained packet flow"),
        (20, 30, "Fast strokes + curves — stress latency"),
        (30, 40, "Normal writing — check if latency recovered"),
        (40, 50, "Idle + air movement — check false updates"),
        (50, 60, "Mixed writing + live viewing — final simulation"),
    ]
    total_min = duration_s // 60
    print("[TEST1] Phase plan:")
    for start_m, end_m, desc in phases:
        if end_m <= total_min or start_m < total_min:
            actual_end = min(end_m, total_min)
            print(f"  {start_m:>3}–{actual_end:<3}min  {desc}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Classroom Simulation Test — Step 2 (benchmark/run_test1_classroom.py)"
    )
    parser.add_argument(
        "--duration", type=int, default=3600,
        help="Test duration in seconds (default: 3600 = 60 min)",
    )
    parser.add_argument(
        "--sample-interval", type=int, default=5,
        help="CPU/RAM sampling interval in seconds (default: 5)",
    )
    args = parser.parse_args()

    if args.duration < 1:
        print("[TEST1] Error: --duration must be at least 1 second.")
        sys.exit(1)

    run_classroom_test(duration_s=args.duration, sample_interval=args.sample_interval)


if __name__ == "__main__":
    main()
