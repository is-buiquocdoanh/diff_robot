import os
import shutil
import struct
import zlib
from pathlib import Path

import numpy as np
import pytest

from a3_web.maps import (MapError, MapStore, encode_indexed_png, grid_to_indices,
                         read_pgm)

REAL_MAP = Path(__file__).resolve().parents[2] / 'a3_maps'


def make_map(root, name, w=8, h=6):
    d = root / name
    d.mkdir(parents=True)
    pix = np.full((h, w), 205, dtype=np.uint8)
    pix[0, :] = 0        # hàng trên cùng: vật cản
    pix[2:4, 2:6] = 254  # vùng trống
    (d / f'{name}.pgm').write_bytes(b'P5\n# c\n%d %d\n255\n' % (w, h) + pix.tobytes())
    (d / f'{name}.yaml').write_text(
        f'image: {name}.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.0, -2.0, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def test_png_roundtrip():
    idx = np.array([[0, 1, 2], [2, 1, 0]], dtype=np.uint8)
    png = encode_indexed_png(idx)
    pos, idat, size = 8, b'', None
    while pos < len(png):
        (length,) = struct.unpack('>I', png[pos:pos + 4])
        tag, data = png[pos + 4:pos + 8], png[pos + 8:pos + 8 + length]
        (crc,) = struct.unpack('>I', png[pos + 8 + length:pos + 12 + length])
        assert crc == zlib.crc32(tag + data) & 0xFFFFFFFF
        if tag == b'IHDR':
            size = struct.unpack('>II', data[:8])
        if tag == b'IDAT':
            idat += data
        pos += 12 + length
    assert size == (3, 2)
    raw = np.frombuffer(zlib.decompress(idat), dtype=np.uint8).reshape(2, 4)
    assert (raw[:, 0] == 0).all()
    assert (raw[:, 1:] == idx).all()


def test_grid_to_indices_flips_and_thresholds():
    # hàng 0 của grid = y thấp nhất -> phải xuống dưới cùng của ảnh
    data = [0, 100, -1, 50]  # 2x2
    idx = grid_to_indices(data, 2, 2)
    assert idx.tolist() == [[1, 1], [0, 2]]   # (-1)->unknown, 50->unknown, 0->free, 100->occ


def test_store_list_info_image(tmp_path):
    make_map(tmp_path, 'room1')
    make_map(tmp_path, 'room2', w=10, h=4)
    (tmp_path / 'not_a_map').mkdir()
    store = MapStore(tmp_path)
    maps = store.list_maps()
    assert [m['name'] for m in maps] == ['room1', 'room2']
    assert maps[0]['width'] == 8 and maps[0]['size_m'] == [0.4, 0.3]
    assert '_image_path' not in maps[0]
    png = store.image_png('room1')
    assert png.startswith(b'\x89PNG')
    assert store.image_png('room1') is png  # cache


def test_store_rejects_bad_names(tmp_path):
    store = MapStore(tmp_path)
    for bad in ('../etc', 'a b', '', 'x' * 41, 'a/b'):
        with pytest.raises(MapError):
            store.map_dir(bad)


def test_waypoints_crud(tmp_path):
    make_map(tmp_path, 'room1')
    store = MapStore(tmp_path)
    wp = store.add_waypoint('room1', {'name': 'Bàn 01', 'type': 'table', 'x': 1.5, 'y': -0.25, 'theta': 1.57})
    assert wp['type'] == 'TABLE' and len(wp['id']) == 8
    with pytest.raises(MapError):
        store.add_waypoint('room1', {'name': 'Bàn 01', 'x': 0, 'y': 0})       # trùng tên
    with pytest.raises(MapError):
        store.add_waypoint('room1', {'name': 'X', 'x': float('nan'), 'y': 0})  # NaN
    with pytest.raises(MapError):
        store.add_waypoint('room1', {'name': 'X', 'type': 'BAD', 'x': 0, 'y': 0})
    upd = store.update_waypoint('room1', wp['id'], {'name': 'Bàn 1', 'x': 2})
    assert upd['name'] == 'Bàn 1' and upd['x'] == 2 and upd['y'] == -0.25
    assert store.find_waypoint('room1', wp['id'])['name'] == 'Bàn 1'
    assert store.info('room1')['waypoints'] == 1
    store.delete_waypoint('room1', wp['id'])
    assert store.get_waypoints('room1') == []
    with pytest.raises(MapError):
        store.delete_waypoint('room1', wp['id'])


def test_delete_and_zip(tmp_path):
    make_map(tmp_path, 'room1')
    store = MapStore(tmp_path)
    assert len(store.zip_bytes('room1')) > 100
    store.delete('room1')
    assert not store.exists('room1')
    with pytest.raises(MapError):
        store.delete('room1')


@pytest.mark.skipif(not (REAL_MAP / 'map1' / 'map1.pgm').exists(), reason='không có a3_maps/map1')
def test_real_user_map(tmp_path):
    shutil.copytree(REAL_MAP / 'map1', tmp_path / 'map1')
    store = MapStore(tmp_path)
    info = store.info('map1')
    w, h, pix = read_pgm(tmp_path / 'map1' / 'map1.pgm')
    assert (info['width'], info['height']) == (w, h) and pix.shape == (h, w)
    assert info['has_posegraph'] is True
    assert store.image_png('map1').startswith(b'\x89PNG')
