"""Quản lý thư mục bản đồ + waypoint (không phụ thuộc ROS, test được độc lập).

Layout mỗi bản đồ (khớp map_saver_cli và src/a3_maps/map1 có sẵn):

    <maps_dir>/<tên>/<tên>.yaml       metadata Nav2 (image, resolution, origin...)
    <maps_dir>/<tên>/<tên>.pgm        ảnh occupancy grid
    <maps_dir>/<tên>/<tên>.posegraph  (tuỳ chọn) slam_toolbox serialize_map
    <maps_dir>/<tên>/waypoints.json   waypoint do web quản lý
"""
import io
import json
import math
import os
import re
import shutil
import struct
import threading
import uuid
import zipfile
import zlib
from pathlib import Path

import numpy as np
import yaml

NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,40}$')
WAYPOINT_TYPES = ('TABLE', 'KITCHEN', 'DOCK', 'POINT')

# Bảng màu PNG hiển thị: 0=trống, 1=chưa biết, 2=vật cản.
PALETTE = [(255, 255, 255), (226, 232, 240), (30, 41, 59)]


class MapError(Exception):
    """Lỗi nghiệp vụ (tên sai, map không tồn tại...) - message hiển thị được cho người dùng."""


# --------------------------------------------------------------------------- PNG
def _chunk(tag, data):
    return (struct.pack('>I', len(data)) + tag + data
            + struct.pack('>I', zlib.crc32(tag + data) & 0xFFFFFFFF))


def encode_indexed_png(idx, palette=PALETTE):
    """idx: mảng uint8 (h, w) chứa chỉ số bảng màu, hàng 0 = trên cùng."""
    h, w = idx.shape
    raw = np.zeros((h, w + 1), dtype=np.uint8)  # mỗi hàng có 1 byte filter = 0
    raw[:, 1:] = idx
    plte = b''.join(bytes(c) for c in palette)
    return (b'\x89PNG\r\n\x1a\n'
            + _chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 3, 0, 0, 0))
            + _chunk(b'PLTE', plte)
            + _chunk(b'IDAT', zlib.compress(raw.tobytes(), 6))
            + _chunk(b'IEND', b''))


def grid_to_indices(data, width, height):
    """OccupancyGrid.data (row 0 = y thấp nhất) -> chỉ số palette, row 0 = trên cùng."""
    arr = np.asarray(data, dtype=np.int8).reshape(height, width)
    idx = np.ones((height, width), dtype=np.uint8)  # chưa biết
    idx[(arr >= 0) & (arr <= 25)] = 0
    idx[arr >= 65] = 2
    return idx[::-1]


# --------------------------------------------------------------------------- PGM
def read_pgm(path, header_only=False):
    """Đọc PGM P5 8-bit. Trả về (width, height, mảng uint8 (h,w) hoặc None nếu header_only)."""
    with open(path, 'rb') as f:
        data = f.read(512 if header_only else -1)
    tokens = []
    i, n = 0, len(data)
    while len(tokens) < 4:
        while i < n and data[i:i + 1].isspace():
            i += 1
        if i >= n:
            raise MapError(f'File PGM hỏng: {os.path.basename(path)}')
        if data[i:i + 1] == b'#':
            while i < n and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < n and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5':
        raise MapError('Chỉ hỗ trợ ảnh bản đồ PGM (P5)')
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise MapError('Chỉ hỗ trợ PGM 8-bit')
    if header_only:
        return w, h, None
    i += 1  # đúng 1 ký tự trắng sau maxval
    pix = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=i).reshape(h, w)
    if maxval != 255:
        pix = (pix.astype(np.uint16) * 255 // maxval).astype(np.uint8)
    return w, h, pix


def pgm_to_indices(pix, negate, occupied_thresh, free_thresh):
    """Quy đổi pixel PGM -> chỉ số palette đúng ngữ nghĩa nav2_map_server (trinary)."""
    p = (pix.astype(np.float32) / 255.0) if negate else ((255 - pix.astype(np.float32)) / 255.0)
    idx = np.ones(pix.shape, dtype=np.uint8)
    idx[p < free_thresh] = 0
    idx[p > occupied_thresh] = 2
    return idx  # hàng 0 của PGM đã là hàng trên cùng


# --------------------------------------------------------------------- waypoint
def _finite(value, name):
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise MapError(f'{name} phải là số')
    if not math.isfinite(v) or abs(v) > 1e5:
        raise MapError(f'{name} không hợp lệ')
    return v


def clean_waypoint(data, base=None):
    """Kiểm tra + chuẩn hóa waypoint. base: waypoint cũ (khi cập nhật từng phần)."""
    wp = dict(base or {})
    if 'name' in data or not base:
        name = str(data.get('name', '')).strip()
        if not (1 <= len(name) <= 40):
            raise MapError('Tên điểm phải dài 1-40 ký tự')
        wp['name'] = name
    if 'type' in data or not base:
        wtype = str(data.get('type', 'POINT')).upper()
        if wtype not in WAYPOINT_TYPES:
            raise MapError(f'Loại điểm phải thuộc {WAYPOINT_TYPES}')
        wp['type'] = wtype
    for key in ('x', 'y'):
        if key in data or not base:
            wp[key] = round(_finite(data.get(key), key), 3)
    if 'theta' in data or not base:
        wp['theta'] = round(_finite(data.get('theta', 0.0), 'theta'), 4)
    return wp


MAX_ROUTE_STEPS = 50


def clean_route(data, valid_ids, base=None):
    """Kiểm tra + chuẩn hóa lộ trình: tên, danh sách id waypoint theo thứ tự (được lặp lại), loop."""
    rt = dict(base or {})
    if 'name' in data or not base:
        name = str(data.get('name', '')).strip()
        if not (1 <= len(name) <= 40):
            raise MapError('Tên lộ trình phải dài 1-40 ký tự')
        rt['name'] = name
    if 'steps' in data or not base:
        steps = data.get('steps')
        if not isinstance(steps, list) or not steps:
            raise MapError('Lộ trình cần ít nhất 1 điểm')
        if len(steps) > MAX_ROUTE_STEPS:
            raise MapError(f'Lộ trình tối đa {MAX_ROUTE_STEPS} điểm')
        for sid in steps:
            if sid not in valid_ids:
                raise MapError('Lộ trình chứa điểm không tồn tại (có thể đã bị xóa)')
        rt['steps'] = list(steps)
    if 'loop' in data or not base:
        rt['loop'] = bool(data.get('loop', False))
    return rt


# ------------------------------------------------------------------------ store
class MapStore:
    def __init__(self, maps_dir):
        self.dir = Path(os.path.expanduser(str(maps_dir)))
        self._lock = threading.RLock()
        self._png_cache = {}  # name -> (mtime, bytes)

    # -- đường dẫn
    def _check_name(self, name):
        if not isinstance(name, str) or not NAME_RE.match(name):
            raise MapError('Tên bản đồ chỉ gồm chữ/số/_/- (tối đa 40 ký tự)')
        return name

    def map_dir(self, name):
        return self.dir / self._check_name(name)

    def yaml_path(self, name):
        return self.map_dir(name) / f'{name}.yaml'

    def exists(self, name):
        try:
            return self.yaml_path(name).is_file()
        except MapError:
            return False

    # -- thông tin
    def info(self, name):
        ypath = self.yaml_path(name)
        if not ypath.is_file():
            raise MapError(f'Không tìm thấy bản đồ "{name}"')
        try:
            meta = yaml.safe_load(ypath.read_text()) or {}
            res = float(meta['resolution'])
            origin = [float(v) for v in meta.get('origin', [0, 0, 0])]
            image = str(meta['image'])
        except Exception:
            raise MapError(f'File yaml của bản đồ "{name}" không hợp lệ')
        img_path = (ypath.parent / image)
        if img_path.suffix.lower() != '.pgm' or not img_path.is_file():
            raise MapError(f'Bản đồ "{name}" thiếu file ảnh .pgm hợp lệ')
        w, h, _ = read_pgm(img_path, header_only=True)
        return {
            'name': name,
            'resolution': res,
            'origin': origin,
            'width': w,
            'height': h,
            'size_m': [round(w * res, 2), round(h * res, 2)],
            'mtime': int(max(ypath.stat().st_mtime, img_path.stat().st_mtime)),
            'has_posegraph': (ypath.parent / f'{name}.posegraph').is_file(),
            'waypoints': len(self.get_waypoints(name)),
            'routes': len(self.get_routes(name)),
            'negate': int(meta.get('negate', 0)),
            'occupied_thresh': float(meta.get('occupied_thresh', 0.65)),
            'free_thresh': float(meta.get('free_thresh', 0.25)),
            '_image_path': str(img_path),
        }

    def list_maps(self):
        out = []
        if not self.dir.is_dir():
            return out
        for entry in sorted(self.dir.iterdir()):
            if entry.is_dir() and NAME_RE.match(entry.name) and (entry / f'{entry.name}.yaml').is_file():
                try:
                    info = self.info(entry.name)
                except MapError:
                    continue
                info.pop('_image_path', None)
                out.append(info)
        return out

    def image_png(self, name):
        info = self.info(name)
        with self._lock:
            cached = self._png_cache.get(name)
            if cached and cached[0] == info['mtime']:
                return cached[1]
            w, h, pix = read_pgm(info['_image_path'])
            idx = pgm_to_indices(pix, info['negate'], info['occupied_thresh'], info['free_thresh'])
            png = encode_indexed_png(idx)
            self._png_cache[name] = (info['mtime'], png)
            return png

    def delete(self, name):
        d = self.map_dir(name)
        if not d.is_dir():
            raise MapError(f'Không tìm thấy bản đồ "{name}"')
        with self._lock:
            shutil.rmtree(d)
            self._png_cache.pop(name, None)

    def zip_bytes(self, name):
        d = self.map_dir(name)
        if not d.is_dir():
            raise MapError(f'Không tìm thấy bản đồ "{name}"')
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, 'w', zipfile.ZIP_DEFLATED) as z:
            for f in sorted(d.iterdir()):
                if f.is_file():
                    z.write(f, arcname=f'{name}/{f.name}')
        return buf.getvalue()

    # -- waypoint
    def _wp_path(self, name):
        return self.map_dir(name) / 'waypoints.json'

    def get_waypoints(self, name):
        path = self._wp_path(name)
        with self._lock:
            if not path.is_file():
                return []
            try:
                data = json.loads(path.read_text())
                return data if isinstance(data, list) else []
            except (OSError, ValueError):
                return []

    def _save_waypoints(self, name, wps):
        path = self._wp_path(name)
        tmp = path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(wps, ensure_ascii=False, indent=2))
        os.replace(tmp, path)

    def add_waypoint(self, name, data):
        if not self.exists(name):
            raise MapError(f'Không tìm thấy bản đồ "{name}"')
        wp = clean_waypoint(data)
        wp['id'] = uuid.uuid4().hex[:8]
        with self._lock:
            wps = self.get_waypoints(name)
            if any(w.get('name') == wp['name'] for w in wps):
                raise MapError(f'Đã có điểm tên "{wp["name"]}"')
            wps.append(wp)
            self._save_waypoints(name, wps)
        return wp

    def update_waypoint(self, name, wid, data):
        with self._lock:
            wps = self.get_waypoints(name)
            for i, w in enumerate(wps):
                if w.get('id') == wid:
                    new = clean_waypoint(data, base=w)
                    if any(o.get('name') == new['name'] and o.get('id') != wid for o in wps):
                        raise MapError(f'Đã có điểm tên "{new["name"]}"')
                    wps[i] = new
                    self._save_waypoints(name, wps)
                    return new
        raise MapError('Không tìm thấy điểm')

    def delete_waypoint(self, name, wid):
        with self._lock:
            wps = self.get_waypoints(name)
            kept = [w for w in wps if w.get('id') != wid]
            if len(kept) == len(wps):
                raise MapError('Không tìm thấy điểm')
            self._save_waypoints(name, kept)
            routes = self.get_routes(name)   # gỡ điểm đã xóa khỏi mọi lộ trình
            changed = False
            for rt in routes:
                steps = [sid for sid in rt.get('steps', []) if sid != wid]
                if len(steps) != len(rt.get('steps', [])):
                    rt['steps'] = steps
                    changed = True
            if changed:
                self._save_routes(name, routes)

    def find_waypoint(self, name, wid):
        for w in self.get_waypoints(name):
            if w.get('id') == wid:
                return w
        return None

    # -- lộ trình
    def _routes_path(self, name):
        return self.map_dir(name) / 'routes.json'

    def get_routes(self, name):
        path = self._routes_path(name)
        with self._lock:
            if not path.is_file():
                return []
            try:
                data = json.loads(path.read_text())
                return data if isinstance(data, list) else []
            except (OSError, ValueError):
                return []

    def _save_routes(self, name, routes):
        path = self._routes_path(name)
        tmp = path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(routes, ensure_ascii=False, indent=2))
        os.replace(tmp, path)

    def _valid_wp_ids(self, name):
        return {w.get('id') for w in self.get_waypoints(name)}

    def add_route(self, name, data):
        if not self.exists(name):
            raise MapError(f'Không tìm thấy bản đồ "{name}"')
        with self._lock:
            rt = clean_route(data, self._valid_wp_ids(name))
            routes = self.get_routes(name)
            if any(r.get('name') == rt['name'] for r in routes):
                raise MapError(f'Đã có lộ trình tên "{rt["name"]}"')
            rt['id'] = uuid.uuid4().hex[:8]
            routes.append(rt)
            self._save_routes(name, routes)
        return rt

    def update_route(self, name, rid, data):
        with self._lock:
            routes = self.get_routes(name)
            for i, r in enumerate(routes):
                if r.get('id') == rid:
                    new = clean_route(data, self._valid_wp_ids(name), base=r)
                    if any(o.get('name') == new['name'] and o.get('id') != rid for o in routes):
                        raise MapError(f'Đã có lộ trình tên "{new["name"]}"')
                    routes[i] = new
                    self._save_routes(name, routes)
                    return new
        raise MapError('Không tìm thấy lộ trình')

    def delete_route(self, name, rid):
        with self._lock:
            routes = self.get_routes(name)
            kept = [r for r in routes if r.get('id') != rid]
            if len(kept) == len(routes):
                raise MapError('Không tìm thấy lộ trình')
            self._save_routes(name, kept)

    def find_route(self, name, rid):
        return next((r for r in self.get_routes(name) if r.get('id') == rid), None)
