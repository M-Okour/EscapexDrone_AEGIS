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


def test_to_bgr8_from_rgb8_swaps_channels():
    import numpy as np
    from exd_camera.image_utils import to_bgr8
    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    rgb[..., 0] = 200   # R
    rgb[..., 2] = 10    # B
    out = to_bgr8(rgb.tobytes(), 3, 2, 9, 'rgb8')
    assert out.shape == (2, 3, 3)
    assert (out[..., 0] == 10).all() and (out[..., 2] == 200).all()


def test_to_bgr8_handles_row_padding_and_alpha():
    import numpy as np
    from exd_camera.image_utils import to_bgr8
    width, height, step = 2, 2, 12          # 8 bytes of bgra + 4 bytes padding per row
    raw = np.zeros((height, step), dtype=np.uint8)
    raw[:, 0:8] = [1, 2, 3, 255, 1, 2, 3, 255]
    out = to_bgr8(raw.tobytes(), width, height, step, 'bgra8')
    assert out.shape == (2, 2, 3)
    assert out[0, 0].tolist() == [1, 2, 3]


def test_to_bgr8_rejects_unknown_encoding():
    import pytest
    from exd_camera.image_utils import to_bgr8
    with pytest.raises(ValueError):
        to_bgr8(b'\x00' * 4, 2, 2, 2, 'mono8')
