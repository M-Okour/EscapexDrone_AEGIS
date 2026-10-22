import math

import numpy as np

from exd_lidar.geometry import nearest_per_sector, simulate_room_scan


def test_room_scan_at_origin():
    pts = simulate_room_scan(0.0, 0.0, 0.0, half_x=5.0, half_y=3.0, n_rays=360)
    dist = np.hypot(pts[:, 0], pts[:, 1])
    assert math.isclose(dist.min(), 3.0, abs_tol=1e-6)   # side walls
    assert dist.max() <= math.hypot(5.0, 3.0) + 1e-6      # corners


def test_room_scan_near_wall():
    pts = simulate_room_scan(4.0, 0.0, 0.0, half_x=5.0, half_y=3.0)
    obstacles = nearest_per_sector(pts)
    assert math.isclose(obstacles[0][3], 1.0, abs_tol=0.05)


def test_nearest_per_sector_filters_range():
    pts = np.array([[0.1, 0.0, 0.0],      # too close (the drone itself)
                    [2.0, 0.0, 0.0],
                    [0.0, -1.5, 0.0],
                    [50.0, 0.0, 0.0]])    # too far
    obstacles = nearest_per_sector(pts, sectors=4, min_range=0.2, max_range=10.0)
    assert [round(o[3], 2) for o in obstacles] == [1.5, 2.0]


def test_empty_cloud():
    assert nearest_per_sector(np.zeros((0, 3))) == []


def test_scan_to_points_drops_invalid_and_places_points():
    from exd_lidar.geometry import scan_to_points
    ranges = [1.0, float('inf'), 2.0, 0.05, float('nan'), 30.0]
    pts = scan_to_points(ranges, angle_min=0.0, angle_increment=math.pi / 2,
                         range_min=0.15, range_max=16.0)
    assert pts.shape == (2, 3)
    assert np.allclose(pts[0], [1.0, 0.0, 0.0], atol=1e-6)     # 0 rad
    assert np.allclose(pts[1], [-2.0, 0.0, 0.0], atol=1e-6)    # pi rad


def test_scan_to_points_feeds_obstacles():
    from exd_lidar.geometry import scan_to_points
    ranges = [3.0] * 360
    ranges[90] = 0.8
    pts = scan_to_points(ranges, -math.pi, 2 * math.pi / 360, 0.15, 16.0)
    assert math.isclose(nearest_per_sector(pts)[0][3], 0.8, abs_tol=1e-5)


def test_scan_to_points_empty():
    from exd_lidar.geometry import scan_to_points
    assert scan_to_points([], 0.0, 0.1, 0.1, 10.0).shape == (0, 3)
