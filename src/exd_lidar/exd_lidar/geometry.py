"""Pure-Python LiDAR helpers (no ROS) so they can be unit-tested anywhere."""
import math

import numpy as np


def simulate_room_scan(x: float, y: float, yaw: float, half_x: float, half_y: float,
                       n_rays: int = 360, noise: float = 0.0, rng=None) -> np.ndarray:
    """Fake 2D LiDAR scan of a rectangular room centred on the map origin.

    The sensor is at (x, y) with heading `yaw`. Returns an (n_rays, 3) array of
    points in the SENSOR frame (z = 0).
    """
    angles = np.linspace(-math.pi, math.pi, n_rays, endpoint=False)
    world = angles + yaw
    c, s = np.cos(world), np.sin(world)
    with np.errstate(divide='ignore'):
        tx = np.where(c > 0, (half_x - x) / c, np.where(c < 0, (-half_x - x) / c, np.inf))
        ty = np.where(s > 0, (half_y - y) / s, np.where(s < 0, (-half_y - y) / s, np.inf))
    r = np.minimum(tx, ty)
    r = np.clip(r, 0.0, None)
    if noise > 0:
        rng = rng or np.random.default_rng()
        r = r + rng.normal(0.0, noise, size=r.shape)
    return np.stack([r * np.cos(angles), r * np.sin(angles), np.zeros_like(r)], axis=1)


def nearest_per_sector(points: np.ndarray, sectors: int = 8,
                       min_range: float = 0.2, max_range: float = 10.0):
    """Split the horizontal plane into `sectors` slices and keep the closest point in each.

    points: (N, 3) array in the sensor frame.
    Returns a list of (x, y, z, distance) tuples, closest first.

    TODO(lidar): this is a placeholder. Real options: voxel downsample + ground
    removal + Euclidean clustering, or an occupancy grid / OctoMap.
    """
    if points.size == 0:
        return []
    pts = points[np.isfinite(points).all(axis=1)]
    dist = np.hypot(pts[:, 0], pts[:, 1])
    keep = (dist >= min_range) & (dist <= max_range)
    pts, dist = pts[keep], dist[keep]
    if pts.size == 0:
        return []
    ang = np.arctan2(pts[:, 1], pts[:, 0])
    sector = ((ang + math.pi) / (2 * math.pi) * sectors).astype(int) % sectors
    result = []
    for k in range(sectors):
        idx = np.where(sector == k)[0]
        if idx.size == 0:
            continue
        i = idx[np.argmin(dist[idx])]
        result.append((float(pts[i, 0]), float(pts[i, 1]), float(pts[i, 2]), float(dist[i])))
    result.sort(key=lambda p: p[3])
    return result
