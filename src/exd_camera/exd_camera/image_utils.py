"""Pure-Python image helpers (no ROS), so they can be unit-tested anywhere."""
import numpy as np


def make_test_pattern(width: int, height: int, t: float) -> np.ndarray:
    """A moving colour gradient with a white square 'target', BGR uint8 (H, W, 3).

    Used in sim mode so downstream nodes get real-looking images.
    """
    xs = np.linspace(0, 255, width, dtype=np.float32)
    ys = np.linspace(0, 255, height, dtype=np.float32)
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[..., 0] = ((xs[None, :] + t * 40) % 256).astype(np.uint8)  # B
    img[..., 1] = ys[:, None].astype(np.uint8)                       # G
    img[..., 2] = 80                                                 # R

    # bouncing square so a detector has something to find
    size = max(8, min(width, height) // 8)
    cx = int((np.sin(t) * 0.5 + 0.5) * (width - size))
    cy = int((np.cos(t * 0.7) * 0.5 + 0.5) * (height - size))
    img[cy:cy + size, cx:cx + size] = 255
    return img


def find_bright_square(img: np.ndarray, threshold: int = 250):
    """Toy 'detector': bounding box (x, y, w, h) of pure-white pixels, or None.

    TODO(camera): replace with a real detector (YOLO / ArUco / colour blob ...).
    """
    mask = np.all(img >= threshold, axis=2)
    if not mask.any():
        return None
    rows = np.where(mask.any(axis=1))[0]
    cols = np.where(mask.any(axis=0))[0]
    return int(cols[0]), int(rows[0]), int(cols[-1] - cols[0] + 1), int(rows[-1] - rows[0] + 1)


def to_bgr8(data, width: int, height: int, step: int, encoding: str) -> np.ndarray:
    """Turn the raw bytes of a sensor_msgs/Image into a BGR uint8 (H, W, 3) array.

    Handles the encodings colour cameras usually send (bgr8, rgb8, bgra8, rgba8)
    and row padding (step > width * channels). Raises ValueError otherwise.
    Used to feed images from vendor drivers (e.g. RealSense sends rgb8) into
    the rest of the stack, which expects bgr8.
    """
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported encoding {encoding!r}')
    # rclpy gives Image.data as array.array('B'), which supports the buffer protocol
    buf = (np.asarray(data, dtype=np.uint8) if isinstance(data, list)
           else np.frombuffer(data, dtype=np.uint8))
    if buf.size < step * height or step < width * channels:
        raise ValueError('image buffer smaller than width/height/step')
    rows = buf[:step * height].reshape(height, step)[:, :width * channels]
    img = rows.reshape(height, width, channels)[..., :3]
    if encoding.startswith('rgb'):
        img = img[..., ::-1]
    return np.ascontiguousarray(img)
