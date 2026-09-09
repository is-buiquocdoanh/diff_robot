#!/usr/bin/env python3
"""Giảm số tam giác của mesh STL cho nhẹ Gazebo/RViz, KHÔNG làm hỏng hình học.

    python3 scripts/decimate_meshes.py <thư_mục_STL_gốc>

Cần: pip install open3d

VÌ SAO PHẢI DECIMATE THEO TỪNG KHỐI RỜI
---------------------------------------
Chạy simplify_quadric_decimation trên NGUYÊN mesh sẽ làm hỏng khung: thuật toán
chỉ có một ngân sách tam giác chung, nên nó xoá sạch các chi tiết nhỏ mà tinh
(4 thanh nhôm định hình 20x20 có rãnh T giữa tầng 1 và tầng 2) trong khi vẫn
giữ nguyên mấy khối thừa tam giác (2 motor, 123k tam giác mỗi cái cho một vật
95x38x38 mm). Kết quả: mất hẳn một đoạn dài ~44 mm trên 4 thanh đó, sai lệch
tới 25 mm — và sai y hệt ở MỌI mức target từ 60k đến 300k.

Cách đúng: tách mesh thành các khối rời (chassis.STL có 82 khối), rồi cấp ngân
sách cho từng khối theo DIỆN TÍCH BỀ MẶT của chính nó. Khối thưa sẵn thì giữ
nguyên, khối thừa tam giác thì cắt mạnh. Sai lệch giảm từ 25 mm xuống 0.4 mm.
"""
import math
import os
import sys

import numpy as np
import open3d as o3d

# số tam giác cấp cho mỗi cm² bề mặt của một khối
DENSITY = 25.0
# sàn tối thiểu, để khối nhỏ mà nhiều chi tiết không bị bóp bẹp
MIN_TRIS = 200

# tên file gốc -> tên file trong package
TARGETS = {
    'chassis.STL': 'chassis.STL',
    'RP-Lidar.STL': 'RP-Lidar.STL',
    'caster_wheel.STL': 'caster_wheel.STL',
    'left_wheel.STL': 'drive_wheel.STL',
}


def decimate(mesh):
    """Decimate từng khối rời riêng biệt theo ngân sách tỉ lệ diện tích."""
    mesh.remove_duplicated_vertices()
    labels, counts, areas = mesh.cluster_connected_triangles()
    labels = np.asarray(labels)
    counts = np.asarray(counts)
    areas = np.asarray(areas)

    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)

    out = o3d.geometry.TriangleMesh()
    for i in range(len(counts)):
        sub = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(verts),
            o3d.utility.Vector3iVector(tris[labels == i]))
        sub.remove_unreferenced_vertices()
        target = int(max(MIN_TRIS, math.ceil(areas[i] / 100.0 * DENSITY)))
        if target < counts[i]:
            sub = sub.simplify_quadric_decimation(target_number_of_triangles=target)
        out += sub

    out.remove_duplicated_vertices()
    out.remove_degenerate_triangles()
    out.compute_vertex_normals()
    return out


def main(src_dir: str) -> int:
    dst_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'meshes'))

    for src_name, dst_name in TARGETS.items():
        src = os.path.join(src_dir, src_name)
        if not os.path.isfile(src):
            print(f'!! không tìm thấy {src}, bỏ qua')
            continue
        dst = os.path.join(dst_dir, dst_name)

        a = o3d.io.read_triangle_mesh(src)
        b = decimate(o3d.io.read_triangle_mesh(src))
        o3d.io.write_triangle_mesh(dst, b, write_ascii=False)

        # sai lệch bề mặt so với bản gốc
        pts = np.asarray(a.sample_points_uniformly(number_of_points=200000).points)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(b))
        d = scene.compute_distance(o3d.core.Tensor(pts.astype(np.float32))).numpy()

        # bounding box phải giữ nguyên - mọi origin trong URDF tính từ nó
        bb_a = a.get_axis_aligned_bounding_box()
        bb_b = b.get_axis_aligned_bounding_box()
        bb_err = max(np.abs(bb_a.min_bound - bb_b.min_bound).max(),
                     np.abs(bb_a.max_bound - bb_b.max_bound).max())

        print(f'{src_name:18s} -> {dst_name:18s} '
              f'{len(a.triangles):7d} -> {len(b.triangles):6d} tris  '
              f'{os.path.getsize(dst)/1e6:5.1f} MB  '
              f'lệch {d.max():5.3f} mm  bbox lệch {bb_err:5.3f} mm')
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        raise SystemExit(1)
    raise SystemExit(main(sys.argv[1]))
