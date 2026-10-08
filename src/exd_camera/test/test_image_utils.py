from exd_camera.image_utils import find_bright_square, make_test_pattern


def test_pattern_shape():
    img = make_test_pattern(320, 240, 0.0)
    assert img.shape == (240, 320, 3)


def test_detector_finds_square():
    img = make_test_pattern(320, 240, 1.3)
    box = find_bright_square(img)
    assert box is not None
    x, y, w, h = box
    assert w == h == 30  # 240 // 8


def test_detector_empty_image():
    img = make_test_pattern(320, 240, 0.0)
    img[:] = 0
    assert find_bright_square(img) is None
