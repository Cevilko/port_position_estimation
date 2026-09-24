# port_position_estimation - Copyright (C) 2026 Cevilko <lvelicko03@gmail.com>
# SPDX-License-Identifier: AGPL-3.0-only
"""sensor_msgs/Image <-> numpy, without cv_bridge.

cv_bridge is deliberately not used. It is compiled against NumPy 1.x, and the
training venv (the only interpreter here with torch) ships NumPy 2.x; importing
cv_bridge under it does not raise, it *segfaults*. For the 8-bit encodings a
camera actually publishes, the conversion is a reshape, so the dependency buys
nothing and costs the whole process.

Everything here is plain Python and numpy on purpose -- no ROS imports -- so it
can be tested without a running graph.
"""

from __future__ import annotations

import numpy as np

#: Encodings we can unpack, mapped to their channel count.
CHANNELS = {
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
    "mono8": 1,
}

#: Encodings whose channel order needs reversing to reach the BGR that
#: ultralytics expects from a numpy array.
RGB_ORDER = {"rgb8", "rgba8"}


def image_to_bgr(height: int, width: int, step: int, encoding: str, data) -> np.ndarray:
    """Unpack a raw image buffer into an HxWx3 BGR array.

    ``step`` is the row stride in bytes and is not always ``width * channels``:
    publishers may pad rows. Slicing per row rather than reshaping the whole
    buffer is what makes padded images come out undistorted instead of sheared.
    """
    encoding = encoding.lower()
    if encoding not in CHANNELS:
        raise ValueError(
            f"unsupported encoding {encoding!r}; expected one of {sorted(CHANNELS)}"
        )
    channels = CHANNELS[encoding]

    expected = width * channels
    if step < expected:
        raise ValueError(f"step {step} is too small for {width}x{channels}")
    if len(data) < height * step:
        raise ValueError(f"buffer holds {len(data)} bytes, need {height * step}")

    flat = np.frombuffer(data, dtype=np.uint8, count=height * step)
    rows = flat.reshape(height, step)[:, :expected]
    image = rows.reshape(height, width, channels)

    if channels == 1:
        return np.repeat(image, 3, axis=2)
    if channels == 4:
        image = image[:, :, :3]
    if encoding in RGB_ORDER:
        image = image[:, :, ::-1]
    # A frombuffer view is read-only, and ascontiguousarray does NOT fix that:
    # when the slicing above happens to leave the data contiguous (bgr8 with no
    # row padding, the common case) it returns the same read-only array. Force
    # a copy so the result is always writable -- predict() letterboxes in place.
    return np.array(image, dtype=np.uint8, copy=True)


def bgr_to_image_fields(image: np.ndarray):
    """Inverse: the (height, width, step, encoding, data) an Image needs."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"expected HxWx3, got {image.shape}")
    image = np.ascontiguousarray(image, dtype=np.uint8)
    height, width = image.shape[:2]
    return height, width, width * 3, "bgr8", image.tobytes()
