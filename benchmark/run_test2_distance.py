"""
benchmark/run_test2_distance.py — Distance & Interference Reliability Test (Step 3)

Tags each packet_log.csv row with a condition label (e.g. "1m", "3m", "5m",
"interference") and runs conditions sequentially with an operator countdown
between them.  After all conditions finish, prints a per-condition PDR/PLR
summary read from the session's packet_log.csv.

Usage (Raspberry Pi):
    BENCHMARK=1 python -m benchmark.run_test2_distance \\
        --condition 1m:300 --condition 3m:300 --condition 5m:300 \\
        --interference interference:300 --transition 15

Quick smoke test (5 s each, 3 s countdown):
    BENCHMARK=1 python -m benchmark.run_test2_distance \\
        --condition near:5 --condition far:5 --transition 3

Note: The Flask+serial server should be running with BENCHMARK=1 before this
script starts.  Hardware must be positioned at the correct distance before each
condition's countdown reaches zero.
"""

import argparse
import csv
import os
import sys
import threading
import time
from collections import defaultdict

import psutil


def _resource_sampler(logger, stop_event: threading.Event, interval_s: float):
    """Background daemon: samples CPU/RAM every interval_s seconds."""
    while not stop_event.wait(timeout=interval_s):
        cpu = psutil.cpu_percent(interval=None)
        ram = psutil.virtual_memory().used / 1e6
        logger.on_resource_sample(cpu_percent=cpu, ram_mb=ram)


def _countdown(seconds: int, next_label: str):
    """Print a per-second countdown so the operator can reposition hardware."""
    print(f"\n[TEST2] Transition — position hardware for '{next_label}'")
    for i in range(seconds, 0, -1):
        print(f"  Starting in {i:>3}s...   ", end="\r", flush=True)
        time.sleep(1)
    print(f"  Starting '{next_label}' now.             ")


def _run_condition(logger, label: str, duration_s: int):
    """Activate condition tag, wait duration_s, then clear the tag."""
    logger.set_condition(label)
    print(f"[TEST2] ▶ Condition '{label}' — {duration_s}s", flush=True)
    start = time.time()
    try:
        time.sleep(duration_s)
    except KeyboardInterrupt:
        raise
    actual = round(time.time() - start, 1)
    logger.set_condition("")
    print(f"[TEST2] ✓ '{label}' done ({actual}s recorded)")


def _summarize_pdr(session_dir: str):
    """Read packet_log.csv and print a per-condition PDR/PLR table (stdlib csv only)."""
    packet_log = os.path.join(session_dir, "packet_log.csv")
    if not os.path.exists(packet_log):
        print("[SUMMARY] packet_log.csv not found — no summary available.")
        return

    stats: dict = defaultdict(lambda: {"total": 0, "valid": 0})
    with open(packet_log, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cond = row.get("condition", "") or "(untagged)"
            stats[cond]["total"] += 1
            if row.get("crc_valid", "False").strip() == "True":
                stats[cond]["valid"] += 1

    if not stats:
        print("[SUMMARY] No packet rows found.")
        return

    sep = "─" * 60
    print(f"\n[TEST2] {sep}")
    print(f"[TEST2]  Per-Condition PDR / PLR Summary")
    print(f"[TEST2] {sep}")
    print(f"[TEST2]  {'Condition':<22}  {'Packets':>8}  {'PDR%':>7}  {'PLR%':>7}")
    print(f"[TEST2] {sep}")
    for cond, s in sorted(stats.items()):
        total = s["total"]
        pdr = (s["valid"] / total * 100) if total > 0 else 0.0
        plr = 100.0 - pdr
        print(f"[TEST2]  {cond:<22}  {total:>8}  {pdr:>6.1f}%  {plr:>6.1f}%")
    print(f"[TEST2] {sep}\n")


def run_distance_test(
    conditions: list,
    transition_s: int = 10,
    sample_interval: int = 5,
):
    os.environ["BENCHMARK"] = "1"

    from benchmark import get_logger
    logger = get_logger(extra_tags={"test": "distance", "conditions": str(conditions)})

    stop_event = threading.Event()
    sampler = threading.Thread(
        target=_resource_sampler,
        args=(logger, stop_event, float(sample_interval)),
        daemon=True,
        name="bm-resource-sampler",
    )
    sampler.start()

    total_s = sum(d for _, d in conditions)
    print(f"[TEST2] Distance / Interference Reliability Test")
    print(f"[TEST2] Session ID  : {logger.session_id}")
    print(f"[TEST2] Output dir  : {logger.session_dir}")
    print(f"[TEST2] Conditions  : {[c for c, _ in conditions]}")
    print(f"[TEST2] Total time  : ~{total_s + transition_s * (len(conditions) - 1)}s")
    print(f"[TEST2] Transition  : {transition_s}s countdown between conditions")
    print("[TEST2] Press Ctrl+C to abort.\n")

    try:
        for i, (label, duration_s) in enumerate(conditions):
            if i > 0:
                _countdown(transition_s, label)
            _run_condition(logger, label, duration_s)
    except KeyboardInterrupt:
        print("\n[TEST2] Interrupted by user.")
        logger.set_condition("")

    stop_event.set()
    logger.stop()

    _summarize_pdr(logger.session_dir)
    print(f"[TEST2] Done. CSVs saved to: {logger.session_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Distance & Interference Reliability Test — Step 3"
    )
    parser.add_argument(
        "--condition", action="append", dest="conditions",
        metavar="LABEL:SECONDS",
        help=(
            "Test condition as 'label:duration_s'. Repeat for multiple. "
            "Example: --condition 1m:300 --condition 3m:300"
        ),
    )
    parser.add_argument(
        "--transition", type=int, default=10,
        help="Countdown seconds between conditions (default: 10)",
    )
    parser.add_argument(
        "--sample-interval", type=int, default=5,
        help="CPU/RAM sampling interval in seconds (default: 5)",
    )
    args = parser.parse_args()

    if not args.conditions:
        print("[TEST2] No conditions specified. Example:")
        print("  python -m benchmark.run_test2_distance \\")
        print("      --condition 1m:300 --condition 3m:300 --condition 5m:300 \\")
        print("      --transition 15")
        sys.exit(1)

    parsed = []
    for raw in args.conditions:
        parts = raw.split(":", 1)
        if len(parts) != 2 or not parts[1].isdigit():
            print(f"[TEST2] Invalid condition format: '{raw}'. Use LABEL:SECONDS (e.g. 1m:300)")
            sys.exit(1)
        parsed.append((parts[0], int(parts[1])))

    run_distance_test(
        conditions=parsed,
        transition_s=args.transition,
        sample_interval=args.sample_interval,
    )


if __name__ == "__main__":
    main()
