"""Tiny stdlib PNG writer (RGB, 8 bit)."""
import struct
import zlib

import numpy as np


def _chunk(tag, data):
    body = tag + data
    return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)


def write_png(path, rgb):
    """Write an (H, W, 3) uint8 array as a PNG file. Row 0 is the top of the image."""
    a = np.ascontiguousarray(rgb, dtype=np.uint8)
    if a.ndim != 3 or a.shape[2] != 3:
        raise ValueError('write_png expects an (H, W, 3) uint8 array')
    h, w, _ = a.shape
    raw = np.concatenate([np.zeros((h, 1), np.uint8), a.reshape(h, w * 3)], axis=1).tobytes()
    data = (b'\x89PNG\r\n\x1a\n'
            + _chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
            + _chunk(b'IDAT', zlib.compress(raw, 6))
            + _chunk(b'IEND', b''))
    with open(path, 'wb') as f:
        f.write(data)
