"""Read health numbers of the onboard computer (Jetson or any Linux PC). No ROS here.

TODO(jetson): add GPU load (/sys/devices/gpu.0/load or the `jtop` Python API
from jetson-stats), power rails, fan speed, and the current nvpmodel mode.
"""
import glob
import os
import shutil


def read_temperatures(base: str = '/sys/class/thermal') -> dict:
    """{zone_type: temperature_C} for every thermal zone we can read."""
    temps = {}
    for zone in sorted(glob.glob(os.path.join(base, 'thermal_zone*'))):
        try:
            with open(os.path.join(zone, 'type')) as f:
                name = f.read().strip()
            with open(os.path.join(zone, 'temp')) as f:
                temps[name] = int(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            continue
    return temps


def read_memory(path: str = '/proc/meminfo') -> dict:
    """{'total_mb', 'available_mb', 'used_percent'}."""
    info = {}
    try:
        with open(path) as f:
            for line in f:
                key, value = line.split(':', 1)
                info[key] = int(value.split()[0])  # kB
    except (OSError, ValueError):
        return {}
    total = info.get('MemTotal', 0)
    avail = info.get('MemAvailable', 0)
    if not total:
        return {}
    return {'total_mb': total / 1024.0, 'available_mb': avail / 1024.0,
            'used_percent': 100.0 * (1.0 - avail / total)}


def read_load() -> dict:
    try:
        l1, l5, _ = os.getloadavg()
    except OSError:
        return {}
    return {'load_1m': l1, 'load_5m': l5, 'cpus': os.cpu_count() or 1}


def read_disk(path: str = '/') -> dict:
    try:
        u = shutil.disk_usage(path)
    except OSError:
        return {}
    return {'disk_free_gb': u.free / 1e9, 'disk_used_percent': 100.0 * u.used / u.total}
