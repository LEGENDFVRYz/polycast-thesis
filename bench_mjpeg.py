import asyncio
import aiohttp
import time
import sys
from dataclasses import dataclass


@dataclass
class Result:
    client_id: int
    frames: int = 0
    bytes_received: int = 0
    error: str = ""


async def mjpeg_client(client_id, url, duration_sec):
    result = Result(client_id=client_id)
    start = time.time()
    buffer = b""

    timeout = aiohttp.ClientTimeout(
        total=None,
        connect=10,
        sock_read=10
    )

    try:
        async with aiohttp.ClientSession(timeout=timeout) as session:
            async with session.get(url) as response:
                if response.status != 200:
                    result.error = f"HTTP {response.status}"
                    return result

                async for chunk in response.content.iter_chunked(4096):
                    now = time.time()

                    if now - start >= duration_sec:
                        break

                    result.bytes_received += len(chunk)

                    # Count JPEG start markers.
                    buffer += chunk
                    result.frames += buffer.count(b"\xff\xd8")

                    # Keep only a small tail so split markers are still counted.
                    buffer = buffer[-2:]

    except Exception as exc:
        result.error = str(exc)

    return result


async def main():
    if len(sys.argv) < 4:
        print("Usage: python bench_mjpeg.py <url> <clients> <duration_sec>")
        print("Example: python bench_mjpeg.py http://192.168.1.17/stream.mjpg 30 60")
        sys.exit(1)

    url = sys.argv[1]
    clients = int(sys.argv[2])
    duration_sec = int(sys.argv[3])

    print(f"Testing MJPEG stream:")
    print(f"URL: {url}")
    print(f"Clients: {clients}")
    print(f"Duration: {duration_sec}s")
    print()

    start = time.time()

    tasks = [
        mjpeg_client(i + 1, url, duration_sec)
        for i in range(clients)
    ]

    results = await asyncio.gather(*tasks)

    elapsed = time.time() - start

    total_frames = sum(r.frames for r in results)
    total_bytes = sum(r.bytes_received for r in results)
    errors = [r for r in results if r.error]

    total_mbps = (total_bytes * 8) / elapsed / 1_000_000
    avg_fps_per_client = total_frames / clients / elapsed

    print("==== MJPEG BENCHMARK RESULT ====")
    print(f"Elapsed: {elapsed:.2f}s")
    print(f"Clients: {clients}")
    print(f"Total frames received: {total_frames}")
    print(f"Average FPS per client: {avg_fps_per_client:.2f}")
    print(f"Total data received: {total_bytes / 1_000_000:.2f} MB")
    print(f"Approx total bandwidth: {total_mbps:.2f} Mbps")
    print(f"Errors: {len(errors)}")

    if errors:
        print()
        print("Client errors:")
        for r in errors[:10]:
            print(f"Client {r.client_id}: {r.error}")

    print()
    print("Per-client summary:")
    for r in results:
        client_fps = r.frames / elapsed
        client_mbps = (r.bytes_received * 8) / elapsed / 1_000_000
        status = "OK" if not r.error else f"ERROR: {r.error}"
        print(
            f"Client {r.client_id:02d}: "
            f"{client_fps:.2f} fps, "
            f"{client_mbps:.2f} Mbps, "
            f"{status}"
        )


if __name__ == "__main__":
    asyncio.run(main())