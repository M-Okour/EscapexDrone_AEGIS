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


def scan_to_points(ranges, angle_min: float, angle_increment: float,
                   range_min: float, range_max: float) -> np.ndarray:
    """Convert a 2D sensor_msgs/LaserScan into an (N, 3) array of points (z = 0).

    Drops invalid returns (inf / nan / outside [range_min, range_max]), which
    RPLIDARs report for directions where nothing was hit.
    """
    r = np.asarray(ranges, dtype=np.float32)
    if r.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    angles = angle_min + angle_increment * np.arange(r.size, dtype=np.float32)
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, angles = r[ok], angles[ok]
    return np.stack([r * np.cos(angles), r * np.sin(angles), np.zeros_like(r)],
                    axis=1).astype(np.float32)

def filter_lidar_points(points: np.ndarray,
                        min_range: float = 0.2,
                        max_range: float = 10.0) -> np.ndarray:
    """Remove invalid and out-of-range LiDAR measurements."""
    pts = np.asarray(points, dtype=np.float32)

    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float32)

    pts = pts.reshape(-1, 3)

    valid = np.isfinite(pts).all(axis=1)
    pts = pts[valid]

    distances = np.hypot(pts[:, 0], pts[:, 1])

    in_range = (
        (distances >= min_range) &
        (distances <= max_range)
    )

    return pts[in_range]


def cluster_lidar_points(points: np.ndarray,
                         tolerance: float = 0.3,
                         min_points: int = 3) -> list:
    """Group nearby LiDAR points into obstacle clusters."""
    if tolerance <= 0 or min_points < 1:
        raise ValueError("Invalid clustering parameters")

    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]

    visited = np.zeros(len(pts), dtype=bool)
    clusters = []

    for i in range(len(pts)):
        if visited[i]:
            continue

        visited[i] = True
        group = [i]
        queue = [i]

        while queue:
            current = queue.pop()
            distances = np.linalg.norm(
                pts[:, :2] - pts[current, :2], axis=1
            )

            neighbors = np.where(
                (distances <= tolerance) & (~visited)
            )[0]

            for neighbor in neighbors:
                visited[neighbor] = True
                group.append(int(neighbor))
                queue.append(int(neighbor))

        if len(group) >= min_points:
            clusters.append(pts[group])

    return clusters


def calculate_min_distance(points: np.ndarray) -> float:
    """Return nearest LiDAR distance, or 0.0 if no valid points exist."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)

    if len(pts) == 0:
        return 0.0

    valid = pts[np.isfinite(pts).all(axis=1)]

    if len(valid) == 0:
        return 0.0

    distances = np.hypot(valid[:, 0], valid[:, 1])
    return float(np.min(distances))


def segment_lidar_scan(points: np.ndarray,
                       gap_threshold: float = 0.3,
                       min_points: int = 3) -> list:
    """Segment consecutive, angularly ordered LiDAR points by spatial gaps."""
    if gap_threshold <= 0 or min_points < 1:
        raise ValueError("Invalid segmentation parameters")

    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)

    if len(pts) == 0:
        return []

    clusters = []
    current = []

    for point in pts:
        if not np.isfinite(point).all():
            if len(current) >= min_points:
                clusters.append(np.asarray(current, dtype=np.float32))
            current = []
            continue

        if current:
            gap = np.linalg.norm(point[:2] - current[-1][:2])
            if gap > gap_threshold:
                if len(current) >= min_points:
                    clusters.append(np.asarray(current, dtype=np.float32))
                current = []

        current.append(point)

    if len(current) >= min_points:
        clusters.append(np.asarray(current, dtype=np.float32))

    return clusters


def detect_lidar_corners(points: np.ndarray,
                         window: int = 5,
                         angle_threshold: float = 30.0) -> list:
    """Find corner indices in an ordered 2D LiDAR point sequence."""
    if window < 1 or not 0 < angle_threshold < 180:
        raise ValueError("Invalid corner detection parameters")

    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)

    if len(pts) <= 2 * window:
        return []

    if not np.isfinite(pts).all():
        raise ValueError("Corner detection requires finite points")

    xy = pts[:, :2]

    before = xy[window:-window] - xy[:-2 * window]
    after = xy[2 * window:] - xy[window:-window]

    dot = np.sum(before * after, axis=1)
    cross = before[:, 0] * after[:, 1] - before[:, 1] * after[:, 0]

    turns = np.abs(np.arctan2(cross, dot))
    candidates = np.where(
        turns > np.deg2rad(angle_threshold)
    )[0] + window

    if len(candidates) == 0:
        return []

    groups = np.split(
        candidates,
        np.where(np.diff(candidates) > 1)[0] + 1
    )

    return [
        int(group[np.argmax(turns[group - window])])
        for group in groups if len(group) > 0
    ]


def split_lidar_at_corners(points: np.ndarray,
                           window: int = 5,
                           angle_threshold: float = 30.0) -> list:
    """Split an ordered, complete 360-degree scan at detected corners."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)

    if len(pts) == 0:
        return []

    corners = detect_lidar_corners(
        pts, window=window, angle_threshold=angle_threshold
    )

    if not corners:
        return [pts]

    segments = np.split(pts, corners)

    # First and last segments meet across the 360-degree scan boundary.
    wrapped = np.vstack((segments[-1], segments[0]))

    return [wrapped] + segments[1:-1]


def segment_lidar_with_corners(
    points: np.ndarray,
    gap_threshold: float = 0.3,
    min_points: int = 3,
    window: int = 5,
    angle_threshold: float = 30.0,
) -> list:
    """Segment ordered LiDAR points at gaps, invalid readings, and corners."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)

    sections = segment_lidar_scan(
        pts,
        gap_threshold=gap_threshold,
        min_points=min_points,
    )

    segments = []
    for section in sections:
        corners = detect_lidar_corners(
            section,
            window=window,
            angle_threshold=angle_threshold,
        )

        boundaries = [0] + corners + [len(section)]

        for start, end in zip(boundaries[:-1], boundaries[1:]):
            segment = section[start:end]
            if len(segment) >= min_points:
                segments.append(segment)

    # Join the first and last segments when the 360-degree scan
    # boundary connects adjacent points on the same surface.
    if (
        len(segments) >= 2
        and len(pts) >= 2
        and np.isfinite(pts[0]).all()
        and np.isfinite(pts[-1]).all()
        and np.linalg.norm(pts[0, :2] - pts[-1, :2]) <= gap_threshold
    ):
        segments[0] = np.vstack((segments[-1], segments[0]))
        segments.pop()

    return segments


def scan_to_points_with_gaps(
    ranges,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> np.ndarray:
    """Convert LaserScan ranges to XYZ while preserving invalid scan positions."""
    distances = np.asarray(ranges, dtype=np.float32)
    angles = angle_min + np.arange(len(distances)) * angle_increment

    points = np.column_stack((
        distances * np.cos(angles),
        distances * np.sin(angles),
        np.zeros(len(distances)),
    )).astype(np.float32)

    invalid = (
        ~np.isfinite(distances)
        | (distances < range_min)
        | (distances > range_max)
    )

    points[invalid] = np.nan
    return points
