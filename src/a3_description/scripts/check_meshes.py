#!/usr/bin/env python3
"""Kiểm tra mesh trong package có đúng bằng file STL gốc không.

Lấy mẫu dày trên mặt mesh gốc rồi đo khoảng cách tới mặt mesh trong package.
Nếu ai đó giảm tam giác (decimate) làm mất hình học, script này sẽ chỉ ra ngay
vị trí bị sai.

    python3 scripts/check_meshes.py <thư_mục_STL_gốc>

Cần: pip install open3d

LƯU Ý: đừng decimate chassis.STL bằng quadric decimation. Khung gồm nhiều thanh
nhôm dài; collapse dọc theo thân thanh gần như không tốn "chi phí" quadric nên
thuật toán bóp bẹp chúng trước tiên. Thử nghiệm cho thấy sai lệch tới 25 mm
(mất hẳn một đoạn trên 4 cột ngoài giữa tầng 1 và tầng 2) ở MỌI mức target,
kể cả 300k/463k tam giác, và tham số maximum_error không có tác dụng.
"""
import os
import sys

import numpy as np
import open3d as o3d

# tên file trong package -> tên file gốc
PAIRS = {
    'chassis.STL': 'chassis.STL',
    'RP-Lidar.STL': 'RP-Lidar.STL',
    'caster_wheel.STL': 'caster_wheel.STL',
    'drive_wheel.STL': 'left_wheel.STL',
}

TOL_MM = 1.0


def main(src_dir: str) -> int:
    mesh_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'meshes'))
    bad = 0

    for pkg_name, src_name in PAIRS.items():
        src = os.path.join(src_dir, src_name)
        dst = os.path.join(mesh_dir, pkg_name)
        if not os.path.isfile(src):
            print(f'!! không tìm thấy {src}, bỏ qua')
            continue

        a = o3d.io.read_triangle_mesh(src)
        b = o3d.io.read_triangle_mesh(dst)
        pts = np.asarray(a.sample_points_uniformly(number_of_points=200000).points)

        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(b))
        d = scene.compute_distance(o3d.core.Tensor(pts.astype(np.float32))).numpy()

        worst = float(d.max())
        ok = worst <= TOL_MM
        bad += 0 if ok else 1
        print(f'{pkg_name:18s} {len(a.triangles):7d} -> {len(b.triangles):7d} tris   '
              f'lệch tối đa {worst:6.3f} mm   {"OK" if ok else "SAI"}')
        if not ok:
            far = pts[d > TOL_MM]
            print(f'   vùng sai: x[{far[:,0].min():.1f},{far[:,0].max():.1f}] '
                  f'y[{far[:,1].min():.1f},{far[:,1].max():.1f}] '
                  f'z[{far[:,2].min():.1f},{far[:,2].max():.1f}] mm (hệ toạ độ CAD)')

    return 1 if bad else 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        raise SystemExit(1)
    raise SystemExit(main(sys.argv[1]))
