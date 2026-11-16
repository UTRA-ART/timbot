#!/usr/bin/env python3
"""Regenerate the low-poly robot meshes in this folder from ../*.STL.

Used for the Foxglove view (gazebo_worlds/scripts/world_markers.py), where the
full SolidWorks meshes (~1M triangles, 40+ MB) are slow to download and draw.
Vertex clustering: snap vertices to a grid, drop collapsed/duplicate triangles.
Needs only numpy:  python3 make_lowpoly.py
"""
import os
import struct

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# mesh -> grid size (m). Coarser for big parts; small parts are already light.
PARTS = {
    'chassis_link': 0.012, 'left_wheel_link': 0.01, 'right_wheel_link': 0.01,
    'caster_link': 0.01, 'stand_link': 0.006, 'zed_link': 0.004,
    'top_lidar_link': 0.006, 'bottom_lidar_link': 0.006,
}


def read_stl(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.frombuffer(data, dtype=np.dtype([('n', '<3f4'), ('v', '<3f4', (3,)), ('a', '<u2')]),
                        count=n, offset=84)
    return rec['v'].astype(np.float64)  # (n, 3, 3)


def write_stl(path, tris):
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<3f4', (3,)), ('a', '<u2')]))
    rec['n'], rec['v'] = normals, tris
    with open(path, 'wb') as f:
        f.write(b'timbot low-poly mesh'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def decimate(tris, cell):
    verts = tris.reshape(-1, 3)
    keys = np.floor(verts / cell).astype(np.int64)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    inverse = inverse.reshape(-1)
    # Each cluster's vertex = mean of the vertices snapped into it.
    sums = np.zeros((len(uniq), 3))
    np.add.at(sums, inverse, verts)
    centers = sums / np.bincount(inverse, minlength=len(uniq))[:, None]
    faces = inverse.reshape(-1, 3)
    keep = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    faces = faces[keep]
    _, first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return centers[faces[np.sort(first)]]


if __name__ == '__main__':
    for name, cell in PARTS.items():
        src = read_stl(os.path.join(HERE, '..', f'{name}.STL'))
        out = decimate(src, cell)
        write_stl(os.path.join(HERE, f'{name}.STL'), out)
        print(f'{name}: {len(src)} -> {len(out)} triangles')
