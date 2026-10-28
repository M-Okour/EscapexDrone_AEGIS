
import numpy as np

from exd_lidar.geometry import filter_lidar_points


def test_filter_removes_invalid_points():
    points = np.array([
        [1.0, 0.0, 0.0],
        [np.nan, 0.0, 0.0],
        [np.inf, 1.0, 0.0],
        [2.0, 1.0, 0.0],
    ])

    result = filter_lidar_points(points)

    assert len(result) == 2
    assert np.all(np.isfinite(result))


def test_filter_removes_out_of_range_points():
    points = np.array([
        [0.1, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [12.0, 0.0, 0.0],
    ])

    result = filter_lidar_points(points)

    assert len(result) == 1
    assert np.isclose(result[0, 0], 1.0)


def test_filter_handles_empty_cloud():
    points = np.zeros((0, 3))

    result = filter_lidar_points(points)

    assert result.shape == (0, 3)


def test_filter_preserves_valid_points():
    points = np.array([
        [1.0, 2.0, 0.0],
        [3.0, 1.0, 0.0],
    ])

    result = filter_lidar_points(points)

    np.testing.assert_allclose(result, points)


from exd_lidar.geometry import cluster_lidar_points


def test_clustering_separates_obstacles():
    points = np.array([
        [1.0, 1.0, 0.0],
        [1.1, 1.0, 0.0],
        [1.2, 1.0, 0.0],
        [4.0, 4.0, 0.0],
        [4.1, 4.0, 0.0],
        [4.2, 4.0, 0.0],
    ])

    clusters = cluster_lidar_points(points)

    assert len(clusters) == 2
    assert all(len(cluster) == 3 for cluster in clusters)


def test_clustering_ignores_isolated_points():
    points = np.array([
        [1.0, 1.0, 0.0],
        [1.1, 1.0, 0.0],
        [1.2, 1.0, 0.0],
        [8.0, 8.0, 0.0],
    ])

    clusters = cluster_lidar_points(points)

    assert len(clusters) == 1
    assert len(clusters[0]) == 3


def test_clustering_handles_empty_cloud():
    points = np.zeros((0, 3))

    clusters = cluster_lidar_points(points)

    assert len(clusters) == 0


def test_clustering_detects_obstacle_inside_room():
    from exd_lidar.geometry import simulate_room_scan

    walls = simulate_room_scan(
        0, 0, 0, 5, 3,
        noise=0.01,
        rng=np.random.default_rng(42)
    )

    obstacle = np.array([
        [1.00, 1.0, 0.0],
        [1.05, 1.0, 0.0],
        [1.10, 1.0, 0.0],
        [1.15, 1.0, 0.0],
    ])

    points = np.vstack((walls, obstacle))
    clusters = cluster_lidar_points(points, tolerance=0.16)

    obstacle_found = any(
        len(cluster) == 4
        and np.linalg.norm(
            np.mean(cluster[:, :2], axis=0) - [1.075, 1.0]
        ) < 0.2
        for cluster in clusters
    )

    assert obstacle_found



def test_calculate_min_distance_safety():
    from exd_lidar.geometry import calculate_min_distance

    # No measurements: treat the path as blocked.
    empty = np.zeros((0, 3), dtype=np.float32)
    assert calculate_min_distance(empty) == 0.0

    # Invalid measurements: treat the path as blocked.
    invalid = np.array([[np.nan, 0.0, 0.0]], dtype=np.float32)
    assert calculate_min_distance(invalid) == 0.0

    # Valid measurements: return the nearest distance.
    points = np.array([
        [3.0, 4.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
    ], dtype=np.float32)

    assert np.isclose(calculate_min_distance(points), 1.0)


def test_corner_detection_noisy_room():
    from exd_lidar.geometry import (
        simulate_room_scan,
        detect_lidar_corners,
    )

    for seed in range(5):
        points = simulate_room_scan(
            0, 0, 0, 5, 3,
            noise=0.01,
            rng=np.random.default_rng(seed),
        )

        corners = detect_lidar_corners(points)

        assert corners == [31, 149, 211, 329]


def test_split_noisy_room_into_four_walls():
    from exd_lidar.geometry import (
        simulate_room_scan,
        split_lidar_at_corners,
    )

    for seed in range(5):
        points = simulate_room_scan(
            0, 0, 0, 5, 3,
            noise=0.01,
            rng=np.random.default_rng(seed),
        )

        walls = split_lidar_at_corners(points)

        assert len(walls) == 4
        assert [len(wall) for wall in walls] == [62, 118, 62, 118]
        assert sum(len(wall) for wall in walls) == 360


def test_noisy_circular_obstacle_segmentation():
    from exd_lidar.geometry import simulate_room_scan, split_lidar_at_corners

    angles = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    dx, dy = np.cos(angles), np.sin(angles)

    for seed in range(5):
        points = simulate_room_scan(
            0, 0, 0, 5, 3,
            noise=0.01,
            rng=np.random.default_rng(seed),
        )
        ranges = np.hypot(points[:, 0], points[:, 1])

        projection = 2.0 * dx
        discriminant = projection**2 - (4.0 - 0.4**2)
        hit = (projection > 0) & (discriminant >= 0)

        obstacle_ranges = projection[hit] - np.sqrt(discriminant[hit])
        ranges[hit] = np.minimum(ranges[hit], obstacle_ranges)

        points = np.column_stack((
            ranges * dx,
            ranges * dy,
            np.zeros_like(ranges),
        ))

        segments = split_lidar_at_corners(points)

        assert len(segments) == 8
        assert sum(len(segment) for segment in segments) == 360
        assert min(
            np.min(np.hypot(segment[:, 0], segment[:, 1]))
            for segment in segments
        ) < 1.7


def test_segmentation_handles_missing_measurements():
    from exd_lidar.geometry import (
        simulate_room_scan,
        segment_lidar_with_corners,
    )

    points = simulate_room_scan(
        0, 0, 0, 5, 3,
        noise=0.01,
        rng=np.random.default_rng(42),
    )

    points[80:95] = np.nan

    segments = segment_lidar_with_corners(points)

    assert len(segments) > 0
    assert all(np.isfinite(segment).all() for segment in segments)
    assert sum(len(segment) for segment in segments) == 345
