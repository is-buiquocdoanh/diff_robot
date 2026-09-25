'use strict';
/* A3 Web - giao diện điều khiển robot. Backend: a3_web/server.py (REST + WebSocket /ws). */

// ============================================================ tiện ích
const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const fmt = (v, n = 2) => (v === null || v === undefined || Number.isNaN(v) ? '-' : Number(v).toFixed(n));
const deg = (r) => (r * 180) / Math.PI;
const rad = (d) => (d * Math.PI) / 180;
const wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));
const store = {
    get(k, d) { try { const v = localStorage.getItem('a3.' + k); return v === null ? d : v; } catch { return d; } },
    set(k, v) { try { localStorage.setItem('a3.' + k, v); } catch { /* bỏ qua */ } },
};
let iconTimer = null;
function icons() { // gom nhiều lần render thành 1 lần vẽ lại icon
    if (iconTimer) return;
    iconTimer = requestAnimationFrame(() => { iconTimer = null; try { lucide.createIcons(); } catch { /* chưa nạp */ } });
}
function setText(id, txt) { const el = $(id); if (el && el.textContent !== txt) el.textContent = txt; }
function setClass(el, cls) { if (el && el.className !== cls) el.className = cls; }

let toastTimer = null;
function toast(msg, type = 'info') {
    const t = $('toast');
    $('toast-message').textContent = msg;
    t.className = 'fixed bottom-20 md:bottom-6 right-4 md:right-6 z-50 border px-4 py-3 rounded-xl shadow-xl flex items-center space-x-3 transition-all duration-300 max-w-sm opacity-100 translate-y-0 pointer-events-none '
        + (type === 'error' ? 'bg-red-50 border-red-300 text-red-800' : type === 'ok' ? 'bg-emerald-50 border-emerald-300 text-emerald-800' : 'bg-white border-slate-200 text-slate-900');
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => t.classList.replace('opacity-100', 'opacity-0'), type === 'error' ? 5000 : 3000);
}

async function api(path, method = 'GET', body) {
    const opt = { method, headers: {} };
    if (body !== undefined) { opt.headers['Content-Type'] = 'application/json'; opt.body = JSON.stringify(body); }
    let r;
    try { r = await fetch(path, opt); } catch (e) { throw new Error('Mất kết nối tới robot'); }
    let data = {};
    try { data = await r.json(); } catch { /* không phải JSON */ }
    if (!r.ok || data.ok === false) { const err = new Error(data.error || `Lỗi ${r.status}`); err.status = r.status; throw err; }
    return data;
}
async function act(fn, okMsg) {
    try { const r = await fn(); if (okMsg) toast(okMsg, 'ok'); return r; } catch (e) { toast(e.message, 'error'); return null; }
}

// ============================================================ modal
function modal({ title, body = '', fields = [], okText = 'OK', cancelText = 'Hủy', danger = false, hideCancel = false }) {
    return new Promise((resolve) => {
        const root = $('modal-root');
        const fieldHtml = fields.map((f) => {
            const id = 'mf-' + f.id;
            let input;
            if (f.type === 'select') {
                input = `<select id="${id}" class="field">${f.options.map((o) => `<option value="${esc(o.v)}"${String(o.v) === String(f.value) ? ' selected' : ''}>${esc(o.l)}</option>`).join('')}</select>`;
            } else {
                input = `<input id="${id}" class="field" type="${f.type || 'text'}" value="${esc(f.value ?? '')}"${f.step ? ` step="${f.step}"` : ''}${f.placeholder ? ` placeholder="${esc(f.placeholder)}"` : ''}${f.maxlength ? ` maxlength="${f.maxlength}"` : ''}>`;
            }
            return `<div><label class="text-xs text-slate-600 font-medium" for="${id}">${esc(f.label)}</label>${input}${f.hint ? `<p class="text-[11px] text-slate-500 mt-1">${esc(f.hint)}</p>` : ''}</div>`;
        }).join('');
        root.innerHTML = `
        <div class="fixed inset-0 z-40 bg-slate-900/40 flex items-center justify-center p-4" id="modal-bg">
            <div class="bg-white rounded-2xl shadow-2xl border border-slate-200 w-full max-w-md p-5 space-y-4" role="dialog" aria-modal="true">
                <h3 class="font-bold text-lg text-slate-900">${esc(title)}</h3>
                ${body ? `<div class="text-sm text-slate-600 space-y-2">${body}</div>` : ''}
                <div class="space-y-3">${fieldHtml}</div>
                <div class="flex justify-end gap-2 pt-1">
                    ${hideCancel ? '' : `<button class="btn btn-soft" id="modal-cancel">${esc(cancelText)}</button>`}
                    <button class="btn ${danger ? 'btn-danger' : 'btn-primary'}" id="modal-ok">${esc(okText)}</button>
                </div>
            </div>
        </div>`;
        const close = (val) => { root.innerHTML = ''; document.removeEventListener('keydown', onKey, true); resolve(val); };
        const collect = () => {
            const out = {};
            fields.forEach((f) => { out[f.id] = $('mf-' + f.id).value; });
            return fields.length ? out : true;
        };
        const onKey = (e) => {
            if (e.key === 'Escape') { e.preventDefault(); close(null); }
            else if (e.key === 'Enter' && e.target.tagName !== 'SELECT') { e.preventDefault(); close(collect()); }
        };
        document.addEventListener('keydown', onKey, true);
        $('modal-ok').onclick = () => close(collect());
        const c = $('modal-cancel'); if (c) c.onclick = () => close(null);
        $('modal-bg').addEventListener('mousedown', (e) => { if (e.target.id === 'modal-bg') close(null); });
        const first = root.querySelector('input,select'); if (first) { first.focus(); if (first.select) first.select(); } else { $('modal-ok').focus(); }
        icons();
    });
}
const confirmBox = (title, body, okText = 'Đồng ý', danger = false) => modal({ title, body: `<p>${body}</p>`, okText, danger });
const modalOpen = () => $('modal-root').children.length > 0;

// ============================================================ trạng thái toàn cục
let S = null;                 // trạng thái mới nhất từ server
let maps = [];                // danh sách bản đồ
let mapsInfo = {};            // theo tên
let WP = [];                  // waypoint của bản đồ đang chọn
let lastSelected = undefined; // để phát hiện đổi bản đồ
let ctxSeq = 0;
let tool = 'pan';
const layers = { grid: true, scan: true };
let selectedWp = null;
let activeTab = 'dashboard';
const navTrack = { goalId: 0, d0: null, state: '', announced: '' };
let prevNeedInit = false;
let prevOpError = null;

const MODE_LABEL = { idle: 'IDLE', mapping: 'ĐANG QUÉT SLAM', navigation: 'ĐIỀU HƯỚNG' };
const STACK_LABEL = { stopped: 'Đã dừng', starting: 'Đang khởi động', running: 'Đang chạy', external: 'Chạy ngoài web', stopping: 'Đang tắt', failed: 'Lỗi' };
const STACK_TITLE = { bringup: 'Bringup robot (ESP32, lidar, EKF...)', slam: 'SLAM (slam_toolbox)', nav: 'Điều hướng (AMCL + Nav2)' };
const NAV_LABEL = { idle: 'Chưa có mục tiêu', sending: 'Đang gửi mục tiêu...', active: 'Đang di chuyển', canceling: 'Đang hủy...', succeeded: 'Đã tới nơi', aborted: 'Nav2 dừng - không tới được', canceled: 'Đã hủy', rejected: 'Nav2 từ chối mục tiêu' };
const TASK_LABEL = { queued: 'Chờ', going: 'Đang đi', arrived: 'Đã tới - chờ xác nhận', done: 'Hoàn thành', failed: 'Thất bại', cancelled: 'Đã hủy' };
const TASK_CHIP = { queued: 'chip-gray', going: 'chip-blue', arrived: 'chip-amber', done: 'chip-green', failed: 'chip-red', cancelled: 'chip-gray' };
const WP_TYPE = { TABLE: 'Bàn ăn', KITCHEN: 'Nhà bếp', DOCK: 'Trạm sạc', POINT: 'Điểm' };
const WP_COLOR = { TABLE: '#06b6d4', KITCHEN: '#f59e0b', DOCK: '#8b5cf6', POINT: '#64748b' };

const navBusy = () => S && ['sending', 'active', 'canceling'].includes(S.nav.state);
const navReady = () => S && S.nav.server_ready && S.mode === 'navigation';
const mapFrameOk = () => S && S.pose.valid && S.pose.frame === 'map';

// ============================================================ tab
const NAV_ON = 'bg-blue-50 text-blue-700 border border-blue-200 font-semibold shadow-sm';
const NAV_OFF = 'text-slate-600 hover:bg-slate-100 hover:text-slate-900 font-medium border border-transparent';
function switchTab(tab) {
    activeTab = tab;
    store.set('tab', tab);
    document.querySelectorAll('.tab-content').forEach((el) => el.classList.remove('active'));
    $('tab-' + tab).classList.add('active');
    document.querySelectorAll('.nav-btn').forEach((b) => setClass(b, `nav-btn w-full flex items-center space-x-3 px-3 py-2.5 rounded-lg text-sm transition-all ${b.dataset.tab === tab ? NAV_ON : NAV_OFF}`));
    document.querySelectorAll('.mob-btn').forEach((b) => setClass(b, `mob-btn flex flex-col items-center text-[10px] px-2 py-1 rounded-lg ${b.dataset.tab === tab ? 'text-blue-700 bg-blue-50 font-semibold' : 'text-slate-600'}`));
    if (tab === 'dashboard') setTimeout(resizeCanvas, 30);
    if (tab === 'maps') refreshMaps();
    if (tab === 'system') { renderStacks(true); }
    stopTeleop();
}

// ============================================================ WebSocket
let ws = null;
function setConn(on) {
    setClass($('conn-badge'), `chip ${on ? 'chip-green' : 'chip-red'}`);
    $('conn-badge').innerHTML = `<span class="w-2 h-2 rounded-full ${on ? 'bg-emerald-500 animate-ping' : 'bg-red-500'}"></span><span>${on ? 'Online' : 'Offline'}</span>`;
    if (!on) setText('robot-status-text', 'Mất kết nối');
}
function connectWS() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    ws = new WebSocket(`${proto}://${location.host}/ws`);
    ws.onopen = () => setConn(true);
    ws.onclose = () => { setConn(false); stopTeleopLocal(); setTimeout(connectWS, 1500); };
    ws.onerror = () => { try { ws.close(); } catch { /* đã đóng */ } };
    ws.onmessage = (ev) => {
        let d; try { d = JSON.parse(ev.data); } catch { return; }
        if (d.type !== 'state') return;
        try { onState(d); } catch (e) { console.error('onState', e); }
    };
}

// ============================================================ nhận trạng thái
function onState(d) {
    const prev = S;
    S = d;

    if (d.selected_map !== lastSelected) loadMapContext(d.selected_map);

    // cảnh báo lỗi chuyển chế độ (1 lần)
    const err = d.mode_op.error;
    if (err && err !== prevOpError) toast(err, 'error');
    prevOpError = err;

    // Nav2 sẵn sàng nhưng robot chưa có vị trí trên bản đồ (AMCL chưa nhận vị trí ban đầu) -> hướng dẫn.
    // Kích hoạt theo ĐIỀU KIỆN (không theo thời điểm Nav2 vừa sẵn sàng): TF map cũ của phiên SLAM trước
    // có thể còn "tươi" vài giây rồi mới hết hạn.
    const needInit = navReady() && !mapFrameOk();
    if (needInit && !prevNeedInit) {
        toast('Nav2 đã sẵn sàng. Hãy nhấn-kéo trên bản đồ tại chỗ robot đang đứng để đặt vị trí ban đầu.', 'ok');
        switchTab('dashboard'); setTool('init');
    }
    prevNeedInit = needInit;

    // kết thúc mục tiêu -> thông báo
    const n = d.nav;
    if (n.goal_id !== navTrack.goalId) { navTrack.goalId = n.goal_id; navTrack.d0 = null; navTrack.announced = ''; }
    if (['succeeded', 'aborted', 'canceled', 'rejected'].includes(n.state) && navTrack.announced !== n.state && prev && prev.nav.goal_id === n.goal_id) {
        navTrack.announced = n.state;
        const name = n.goal && n.goal.name ? ` ${n.goal.name}` : '';
        if (n.state === 'succeeded') toast(`Robot đã tới${name}`, 'ok');
        else if (n.state !== 'canceled') toast(`${NAV_LABEL[n.state]}${name ? ' -' + name : ''}`, 'error');
    }

    announceRunEnd(prev);
    renderHeader();
    renderHud();
    renderNavCard();
    renderSensors();
    renderRun();
    renderRoutes();
    renderStacks(false);
    renderSideInfo();
    if (activeTab === 'maps') renderMaps();
}

// ------------------------------------------------------------ header / side
function renderHeader() {
    const modeCls = { idle: 'chip-gray', mapping: 'chip-amber', navigation: 'chip-blue' }[S.mode];
    setClass($('mode-chip'), `chip ${modeCls}`);
    setText('mode-chip', MODE_LABEL[S.mode]);

    const busy = S.mode_op.busy;
    $('op-indicator').classList.toggle('hidden', !busy);
    if (busy) setText('op-text', S.mode_op.message || 'Đang chuyển chế độ...');

    const sel = $('map-select');
    const wantKey = maps.map((m) => m.name).join('|') + '#' + (S.selected_map || '') + '#' + (S.mode !== 'idle');
    if (sel.dataset.key !== wantKey) {
        sel.dataset.key = wantKey;
        sel.innerHTML = (maps.length ? '' : '<option value="">(chưa có bản đồ)</option>')
            + (S.selected_map ? '' : (maps.length ? '<option value="">-- chọn bản đồ --</option>' : ''))
            + maps.map((m) => `<option value="${esc(m.name)}"${m.name === S.selected_map ? ' selected' : ''}>${esc(m.name)}</option>`).join('');
        sel.disabled = S.mode !== 'idle';
        sel.title = S.mode !== 'idle' ? 'Dừng quét/điều hướng để đổi bản đồ đang xem' : '';
    }

    const b = S.battery;
    const bb = $('battery-box');
    if (b && b.percentage !== null) { bb.classList.remove('hidden'); bb.classList.add('flex'); setText('header-battery', `${fmt(b.percentage, 0)}%` + (b.voltage ? ` (${fmt(b.voltage, 1)}V)` : '')); }
    else { bb.classList.add('hidden'); bb.classList.remove('flex'); }

    let cls = 'chip-green', txt = 'Sẵn sàng (IDLE)';
    if (S.estop) { cls = 'chip-red animate-pulse'; txt = 'ĐÃ DỪNG KHẨN CẤP'; }
    else if (busy) { cls = 'chip-amber'; txt = S.mode_op.message || 'Đang chuyển chế độ'; }
    else if (navBusy()) { cls = 'chip-blue'; txt = 'Đang di chuyển' + (S.nav.goal && S.nav.goal.name ? ` → ${S.nav.goal.name}` : ''); }
    else if (S.mode === 'mapping') { cls = 'chip-amber'; txt = 'Đang quét SLAM'; }
    else if (S.mode === 'navigation') { txt = S.nav.server_ready ? (mapFrameOk() ? 'Điều hướng - sẵn sàng' : 'Cần đặt vị trí ban đầu') : 'Nav2 đang khởi động...'; if (!S.nav.server_ready || !mapFrameOk()) cls = 'chip-amber'; }
    else if (!S.stacks.bringup.ready) { cls = 'chip-gray'; txt = 'Chưa bringup robot'; }
    setClass($('status-badge'), `chip ${cls} text-xs px-3 py-1.5`);
    setText('robot-status-text', txt);

    const eb = $('btn-estop');
    setText('estop-label', S.estop ? 'NHẢ E-STOP' : 'E-STOP');
    setClass(eb, `px-3 md:px-4 py-2 text-white font-bold text-xs uppercase tracking-wider rounded-lg shadow-md border flex items-center space-x-2 transition-all active:scale-95 ${S.estop ? 'bg-amber-500 hover:bg-amber-600 border-amber-400 animate-pulse' : 'bg-gradient-to-r from-red-600 to-rose-600 hover:from-red-700 hover:to-rose-700 shadow-red-500/20 border-red-500/30'}`);

    $('mapping-actions').classList.toggle('hidden', S.mode !== 'mapping');
    $('mapping-actions').classList.toggle('flex', S.mode === 'mapping');
}

function renderSideInfo() {
    setText('ft-mode', MODE_LABEL[S.mode]);
    setText('ft-frame', S.pose.valid ? S.pose.frame : '-');
    setText('ft-lidar', S.scan.hz ? `${S.scan.hz} Hz` : 'không có');
    setText('ft-nav', S.nav.server_ready ? 'sẵn sàng' : 'chưa chạy');
}

function renderHud() {
    const p = S.pose, ok = p.valid;
    setText('hud-x', ok ? fmt(p.x) : '-');
    setText('hud-y', ok ? fmt(p.y) : '-');
    setText('hud-theta', ok ? fmt(deg(p.theta), 0) : '-');
    setText('hud-speed', fmt(S.speed.linear));
}

function renderNavCard() {
    const n = S.nav;
    let line = NAV_LABEL[n.state] || n.state;
    if (n.goal && n.state !== 'idle') line += ` → ${n.goal.name || `(${fmt(n.goal.x)}, ${fmt(n.goal.y)})`}`;
    setText('nav-line', line);
    if (n.distance_remaining !== null && navTrack.d0 === null && navBusy()) navTrack.d0 = Math.max(n.distance_remaining, 0.05);
    let pct = 0;
    if (n.state === 'succeeded') pct = 100;
    else if (navBusy() && navTrack.d0 && n.distance_remaining !== null) pct = clamp((1 - n.distance_remaining / navTrack.d0) * 100, 0, 100);
    $('nav-progress').style.width = pct + '%';
    setText('nav-extra', navBusy() && n.distance_remaining !== null ? `Còn ${fmt(n.distance_remaining)} m${n.eta ? ` · ~${Math.round(n.eta)} s` : ''}` : '');
    $('btn-cancel-nav').classList.toggle('hidden', !['sending', 'active'].includes(n.state));
}

function renderSensors() {
    const d = S.scan.min_dist;
    setText('s-obstacle', d === null ? '-' : `${fmt(d)} m`);
    const bar = $('s-obstacle-bar');
    const w = d === null ? 0 : clamp(d / 3, 0, 1) * 100;
    bar.style.width = w + '%';
    setClass(bar, `h-1.5 rounded-full transition-all ${d !== null && d < 0.5 ? 'bg-red-500' : d !== null && d < 1.0 ? 'bg-amber-500' : 'bg-emerald-500'}`);
    setText('s-speed', `${fmt(S.speed.linear)} m/s · ${fmt(S.speed.angular)} rad/s`);
    setText('s-lidar', S.scan.hz ? `${S.scan.hz} Hz` : 'không có dữ liệu');
    setText('sys-maps-dir', S.web.maps_dir);
    setText('sys-limits', `${S.limits.max_linear} m/s · ${S.limits.max_angular} rad/s`);
    setText('sys-robot', `${S.robot.length} × ${S.robot.width} m`);
    const sc = $('sel-controller'); if (sc.value !== S.settings.controller && document.activeElement !== sc) sc.value = S.settings.controller;
    const rl = $('rng-lin'), ra = $('rng-ang');
    if (rl.max !== String(Math.min(S.limits.max_linear, 1))) { rl.max = Math.min(S.limits.max_linear, 1); ra.max = Math.min(S.limits.max_angular, 3); syncTeleopSliders(); }
}

// ============================================================ bản đồ / waypoint (dữ liệu)
async function refreshMaps() {
    try {
        const r = await api('/api/maps');
        maps = r.maps; mapsInfo = {}; maps.forEach((m) => { mapsInfo[m.name] = m; });
        setText('maps-dir', r.maps_dir);
        if (S) { $('map-select').dataset.key = ''; renderHeader(); }
        renderMaps(true);
    } catch (e) { toast(e.message, 'error'); }
}

async function loadMapContext(name) {
    const seq = ++ctxSeq;
    lastSelected = name;
    selectedWp = null; renderWpCard();
    if (name && !mapsInfo[name]) await refreshMaps();
    if (!name) { WP = []; renderWaypoints(); renderQuick(); ROUTES = []; renderRoutes(); return; }
    try {
        const r = await api(`/api/maps/${encodeURIComponent(name)}/waypoints`);
        if (seq !== ctxSeq) return;
        WP = r.waypoints;
    } catch { WP = []; }
    renderWaypoints(); renderQuick();
    await reloadRoutes();
}

async function reloadWaypoints() {
    if (!S || !S.selected_map) return;
    try { WP = (await api(`/api/maps/${encodeURIComponent(S.selected_map)}/waypoints`)).waypoints; } catch { WP = []; }
    if (selectedWp && !WP.find((w) => w.id === selectedWp)) selectedWp = null;
    renderWaypoints(); renderQuick(); renderWpCard();
    await reloadRoutes();
}

async function selectMap(name) {
    if (!name) return;
    await act(() => api('/api/maps/select', 'POST', { name }));
}

// ============================================================ tab Bản đồ
let mapsKey = '';
function renderMaps(force) {
    const key = JSON.stringify([maps, S && S.selected_map, S && S.active_map, S && S.mode]);
    if (!force && key === mapsKey) return;
    mapsKey = key;
    const grid = $('maps-grid');
    if (!maps.length) {
        grid.innerHTML = `<div class="col-span-full text-center text-slate-500 border-2 border-dashed border-slate-300 rounded-xl p-10">Chưa có bản đồ nào. Bấm <b>"Quét bản đồ mới"</b>, lái robot đi hết khu vực rồi bấm <b>"Lưu bản đồ"</b>.</div>`;
        return;
    }
    grid.innerHTML = maps.map((m) => {
        const active = S && S.mode === 'navigation' && S.active_map === m.name;
        const sel = S && S.selected_map === m.name;
        const n = encodeURIComponent(m.name);
        return `
        <div class="bg-white ${active ? 'border-2 border-blue-500 shadow-md' : 'border border-slate-200 shadow-sm'} rounded-xl p-4 flex flex-col justify-between space-y-3">
            <div>
                <div class="flex justify-between items-start mb-2">
                    <span class="chip ${active ? 'chip-blue' : sel ? 'chip-green' : 'chip-gray'}">${active ? 'ĐANG ĐIỀU HƯỚNG' : sel ? 'ĐANG XEM' : 'ĐÃ LƯU'}</span>
                    <span class="text-xs font-mono text-slate-500">${fmt(m.size_m[0], 1)}m × ${fmt(m.size_m[1], 1)}m</span>
                </div>
                <h3 class="font-bold text-base text-slate-900 break-all">${esc(m.name)}</h3>
                <p class="text-xs text-slate-500 mt-1">${m.waypoints} điểm · độ phân giải ${m.resolution} m/px${m.has_posegraph ? ' · có posegraph' : ''}</p>
            </div>
            <div class="h-40 bg-slate-100 rounded-lg border border-slate-200 flex items-center justify-center overflow-hidden">
                <img alt="${esc(m.name)}" loading="lazy" class="w-full h-full object-contain" style="image-rendering:pixelated" src="/api/maps/${n}/image.png?v=${m.mtime}">
            </div>
            <div class="text-[11px] text-slate-500 font-mono">Sửa lần cuối: ${new Date(m.mtime * 1000).toLocaleString('vi-VN')}</div>
            <div class="flex gap-2">
                <button data-act="nav" data-name="${esc(m.name)}" class="btn ${active ? 'btn-soft' : 'btn-primary'} flex-1" ${active ? 'disabled' : ''}><i data-lucide="navigation" class="w-4 h-4"></i>${active ? 'Đang dùng' : 'Điều hướng'}</button>
                <button data-act="view" data-name="${esc(m.name)}" class="btn btn-soft" title="Xem bản đồ này" ${S && S.mode !== 'idle' ? 'disabled' : ''}><i data-lucide="eye" class="w-4 h-4"></i></button>
                <a href="/api/maps/${n}/download" class="btn btn-soft" title="Tải về (.zip)"><i data-lucide="download" class="w-4 h-4"></i></a>
                <button data-act="del" data-name="${esc(m.name)}" class="btn btn-danger" title="Xóa" ${active ? 'disabled' : ''}><i data-lucide="trash-2" class="w-4 h-4"></i></button>
            </div>
        </div>`;
    }).join('');
    icons();
}
$('maps-grid').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-act]');
    if (!b) return;
    const name = b.dataset.name;
    if (b.dataset.act === 'nav') activateMap(name);
    else if (b.dataset.act === 'view') act(() => api('/api/maps/select', 'POST', { name })).then(() => switchTab('dashboard'));
    else if (b.dataset.act === 'del') deleteMap(name);
});

async function ensureBringup() {
    const st = S.stacks.bringup.state;
    if (st === 'stopped') { toast('Đang bật bringup robot...'); const r = await act(() => api('/api/stacks/bringup/start', 'POST')); return !!r; }
    if (st === 'failed') { toast('Bringup đang lỗi - xem log ở tab Hệ thống', 'error'); return false; }
    return true;
}

async function startScan() {
    if (S.mode === 'mapping') { switchTab('dashboard'); return; }
    const ok = await modal({
        title: 'Quét bản đồ mới (SLAM)', okText: 'Bắt đầu quét',
        body: `<ol class="list-decimal ml-5 space-y-1"><li>Web sẽ bật SLAM${S.mode === 'navigation' ? ' và <b>tắt Nav2</b>' : ''}.</li><li>Lái robot chậm quanh khu vực bằng nút điều khiển tay hoặc phím W A S D.</li><li>Khi bản đồ đủ, bấm <b>"Lưu bản đồ"</b>.</li></ol>`,
    });
    if (!ok) return;
    if (!(await ensureBringup())) return;
    if (await act(() => api('/api/mode', 'POST', { mode: 'mapping' }))) switchTab('dashboard');
}

async function endScan() {
    if (!(await confirmBox('Kết thúc quét?', 'Bản đồ chưa lưu sẽ mất. Bạn nên bấm "Lưu bản đồ" trước.', 'Kết thúc quét', true))) return;
    await act(() => api('/api/mode', 'POST', { mode: 'idle' }));
}

function defaultMapName() {
    const d = new Date(), p = (n) => String(n).padStart(2, '0');
    return `map_${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}`;
}
async function saveMapDialog() {
    const v = await modal({
        title: 'Lưu bản đồ', okText: 'Lưu',
        fields: [{ id: 'name', label: 'Tên bản đồ', value: defaultMapName(), maxlength: 40, hint: 'Chỉ chữ, số, gạch dưới (_) và gạch ngang (-).' }],
    });
    if (!v) return;
    await saveMap(v.name.trim(), false);
}
async function saveMap(name, overwrite) {
    toast('Đang lưu bản đồ (có thể mất vài giây)...');
    try {
        const r = await api('/api/maps/save', 'POST', { name, overwrite });
        toast(`Đã lưu bản đồ "${r.name}"${r.posegraph ? ' (kèm posegraph)' : ''}`, 'ok');
        await refreshMaps();
    } catch (e) {
        if (e.status === 409 && await confirmBox('Bản đồ đã tồn tại', `Đã có bản đồ tên "${esc(name)}". Ghi đè?`, 'Ghi đè', true)) return saveMap(name, true);
        if (e.status !== 409) toast(e.message, 'error');
    }
}

async function activateMap(name) {
    if (navBusy() && !(await confirmBox('Đổi bản đồ?', 'Mục tiêu đang chạy sẽ bị hủy.', 'Tiếp tục'))) return;
    if (!(await ensureBringup())) return;
    if (await act(() => api('/api/mode', 'POST', { mode: 'navigation', map: name }), `Đang khởi động Nav2 với "${name}"...`)) switchTab('dashboard');
}

async function deleteMap(name) {
    if (!(await confirmBox('Xóa bản đồ?', `Xóa vĩnh viễn "${esc(name)}" cùng toàn bộ waypoint của nó.`, 'Xóa', true))) return;
    if (await act(() => api(`/api/maps/${encodeURIComponent(name)}`, 'DELETE'), 'Đã xóa bản đồ')) refreshMaps();
}

// ============================================================ tab Waypoints
function wpRow(w) {
    const id = esc(w.id);
    return `<tr class="hover:bg-slate-50 ${selectedWp === w.id ? 'bg-blue-50' : ''}">
        <td class="p-3 font-semibold text-slate-900"><span class="inline-block w-2.5 h-2.5 rounded-full mr-2 align-middle" style="background:${WP_COLOR[w.type] || '#64748b'}"></span>${esc(w.name)}</td>
        <td class="p-3 font-mono text-xs text-slate-500">${esc(WP_TYPE[w.type] || w.type)}</td>
        <td class="p-3 font-mono text-xs text-blue-600 font-semibold">${fmt(w.x)}, ${fmt(w.y)} m</td>
        <td class="p-3 font-mono text-xs text-amber-600 font-semibold">${fmt(deg(w.theta), 0)}°</td>
        <td class="p-3 text-right whitespace-nowrap space-x-1">
            <button data-act="go" data-id="${id}" class="btn btn-primary !px-2.5 !py-1" ${navReady() && !S.estop ? '' : 'disabled'} title="Đi tới điểm này">Đi tới</button>
            <button data-act="init" data-id="${id}" class="btn btn-soft !px-2 !py-1" title="Đặt làm vị trí ban đầu của robot (AMCL)"><i data-lucide="locate-fixed" class="w-3.5 h-3.5"></i></button>
            <button data-act="edit" data-id="${id}" class="btn btn-soft !px-2 !py-1" title="Sửa"><i data-lucide="pencil" class="w-3.5 h-3.5"></i></button>
            <button data-act="del" data-id="${id}" class="btn btn-danger !px-2 !py-1" title="Xóa"><i data-lucide="trash-2" class="w-3.5 h-3.5"></i></button>
        </td></tr>`;
}
function renderWaypoints() {
    const map = S && S.selected_map;
    setText('wp-map-name', map || '(chưa chọn bản đồ)');
    $('wp-body').innerHTML = WP.length ? WP.map(wpRow).join('')
        : `<tr><td colspan="5" class="p-8 text-center text-slate-500">${map ? 'Chưa có điểm nào. Dùng công cụ "Thêm điểm" trên bản đồ hoặc "Lưu vị trí robot hiện tại".' : 'Hãy chọn hoặc quét một bản đồ trước.'}</td></tr>`;
    icons();
}
$('wp-body').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-act]');
    if (!b) return;
    wpAction(b.dataset.act, b.dataset.id);
});

async function wpAction(action, id) {
    const w = WP.find((x) => x.id === id);
    if (!w) return;
    if (action === 'go') goToWp(id);
    else if (action === 'init') { await act(() => api('/api/initialpose', 'POST', { x: w.x, y: w.y, theta: w.theta }), `Đã gửi vị trí ban đầu tại "${w.name}"`); }
    else if (action === 'edit') editWaypoint(w);
    else if (action === 'del') {
        if (await confirmBox('Xóa điểm?', `Xóa "${esc(w.name)}"?`, 'Xóa', true)) {
            if (await act(() => api(`/api/maps/${encodeURIComponent(S.selected_map)}/waypoints/${w.id}`, 'DELETE'), 'Đã xóa điểm')) reloadWaypoints();
        }
    }
}

const TYPE_OPTIONS = Object.entries(WP_TYPE).map(([v, l]) => ({ v, l }));
async function editWaypoint(w) {
    const v = await modal({
        title: `Sửa điểm "${w.name}"`, okText: 'Lưu',
        fields: [
            { id: 'name', label: 'Tên', value: w.name, maxlength: 40 },
            { id: 'type', label: 'Loại', type: 'select', value: w.type, options: TYPE_OPTIONS },
            { id: 'x', label: 'X (m)', type: 'number', step: '0.01', value: w.x },
            { id: 'y', label: 'Y (m)', type: 'number', step: '0.01', value: w.y },
            { id: 'deg', label: 'Hướng (độ)', type: 'number', step: '1', value: Math.round(deg(w.theta)) },
        ],
    });
    if (!v) return;
    const r = await act(() => api(`/api/maps/${encodeURIComponent(S.selected_map)}/waypoints/${w.id}`, 'PUT',
        { name: v.name, type: v.type, x: parseFloat(v.x), y: parseFloat(v.y), theta: rad(parseFloat(v.deg)) }), 'Đã cập nhật điểm');
    if (r) reloadWaypoints();
}

async function createWaypoint(x, y, theta) {
    if (!S.selected_map) { toast('Hãy chọn một bản đồ trước', 'error'); return; }
    if (S.mode === 'mapping') { toast('Hãy lưu bản đồ rồi mới thêm điểm', 'error'); return; }
    const n = WP.filter((w) => w.type === 'TABLE').length + 1;
    const v = await modal({
        title: 'Thêm điểm mới', okText: 'Thêm',
        body: `<p class="font-mono text-xs">(${fmt(x)}, ${fmt(y)}) m · hướng ${fmt(deg(theta), 0)}°</p>`,
        fields: [
            { id: 'name', label: 'Tên', value: `Bàn ${String(n).padStart(2, '0')}`, maxlength: 40 },
            { id: 'type', label: 'Loại', type: 'select', value: 'TABLE', options: TYPE_OPTIONS },
        ],
    });
    if (!v) return;
    const r = await act(() => api(`/api/maps/${encodeURIComponent(S.selected_map)}/waypoints`, 'POST', { name: v.name, type: v.type, x, y, theta }), `Đã thêm "${v.name}"`);
    if (r) { await reloadWaypoints(); selectedWp = r.waypoint.id; renderWpCard(); }
}

function addWaypointHere() {
    if (!mapFrameOk()) { toast('Chưa có vị trí robot trên bản đồ (cần chế độ Điều hướng + vị trí ban đầu)', 'error'); return; }
    createWaypoint(S.pose.x, S.pose.y, S.pose.theta);
}

async function goToWp(id) {
    const w = WP.find((x) => x.id === id);
    if (!w) return;
    if (await act(() => api('/api/nav/goal', 'POST', { waypoint_id: id }), `Đang đi tới ${w.name}...`)) { navTrack.d0 = null; }
}

function renderQuick() {
    const box = $('quick-list');
    if (!WP.length) { box.innerHTML = `<div class="col-span-3 text-xs text-slate-500">Chưa có điểm nào cho bản đồ này.</div>`; return; }
    const dis = navReady() && !S.estop ? '' : 'disabled';
    box.innerHTML = WP.slice(0, 18).map((w) => `
        <button data-id="${esc(w.id)}" ${dis} class="p-2 rounded-lg bg-slate-50 border border-slate-200 hover:border-blue-400 hover:bg-blue-50 text-xs font-semibold text-slate-800 transition text-center shadow-sm disabled:opacity-50 disabled:cursor-not-allowed">
            <div class="text-[10px] font-mono" style="color:${WP_COLOR[w.type] || '#64748b'}">${esc(WP_TYPE[w.type] || w.type)}</div>
            <div class="truncate">${esc(w.name)}</div>
        </button>`).join('');
}
$('quick-list').addEventListener('click', (e) => { const b = e.target.closest('button[data-id]'); if (b) goToWp(b.dataset.id); });

// ============================================================ Lộ trình
let ROUTES = [];              // lộ trình của bản đồ đang chọn
let highlightRoute = null;    // lộ trình được bấm để xem đường đi trên bản đồ
let editor = null;            // {id|null, name, loop, steps:[waypointId,...]} khi đang tạo/sửa
let lastRun = null;

const routePts = (steps) => steps.map((id) => WP.find((w) => w.id === id)).filter(Boolean);
const routeById = (id) => ROUTES.find((r) => r.id === id);

async function reloadRoutes() {
    if (!S || !S.selected_map) { ROUTES = []; renderRoutes(); return; }
    try { ROUTES = (await api(`/api/maps/${encodeURIComponent(S.selected_map)}/routes`)).routes; } catch { ROUTES = []; }
    if (highlightRoute && !routeById(highlightRoute)) highlightRoute = null;
    renderRoutes();
}

function canRun() { return navReady() && mapFrameOk() && !S.estop && !S.run; }
function runHint() {
    if (S.run) return 'Đang có lộ trình chạy - dừng nó trước';
    if (S.estop) return 'E-STOP đang bật';
    if (S.mode !== 'navigation') return 'Vào chế độ Điều hướng (tab Bản đồ → Điều hướng) để chạy lộ trình';
    if (!S.nav.server_ready) return 'Nav2 đang khởi động...';
    if (!mapFrameOk()) return 'Hãy đặt "Vị trí ban đầu" cho robot trên bản đồ';
    return 'Chạy lộ trình này';
}

function stepChips(r) {
    const wps = r.steps.map((id) => WP.find((w) => w.id === id));
    return wps.map((w, i) => `<span class="chip ${w ? 'chip-gray' : 'chip-red'}" style="${w ? `border-left:3px solid ${WP_COLOR[w.type] || '#64748b'}` : ''}"><b class="text-slate-500">${i + 1}</b> ${w ? esc(w.name) : '(đã xóa)'}</span>`).join('<span class="text-slate-400">→</span>')
        + (r.loop ? '<span class="chip chip-blue">↻ lặp</span>' : '');
}

let routesKey = '';
function renderRoutes() {
    const key = JSON.stringify([ROUTES, WP.map((w) => [w.id, w.name, w.type]), highlightRoute, S && [S.run && S.run.route_id, canRun(), S.mode]]);
    if (key === routesKey) return;
    routesKey = key;
    setText('route-count-badge', String(ROUTES.length));
    setText('route-map-name', (S && S.selected_map) || '(chưa chọn bản đồ)');
    const running = S && S.run ? S.run.route_id : null;
    const ok = S && canRun();
    const hint = S ? runHint() : '';

    // ---- tab Lộ trình: thẻ đầy đủ
    $('routes-list').innerHTML = ROUTES.length ? ROUTES.map((r) => `
        <div class="bg-white ${running === r.id ? 'border-2 border-purple-400' : 'border border-slate-200'} rounded-xl p-4 shadow-sm space-y-3">
            <div class="flex justify-between items-start gap-2">
                <div><h3 class="font-bold text-slate-900 break-all">${esc(r.name)}</h3><p class="text-xs text-slate-500">${r.steps.length} điểm${r.loop ? ' · lặp liên tục' : ''}</p></div>
                <div class="flex gap-1.5 shrink-0">
                    <button data-act="run" data-id="${esc(r.id)}" class="btn btn-primary" ${ok ? '' : 'disabled'} title="${esc(hint)}"><i data-lucide="play" class="w-4 h-4"></i>Chạy</button>
                    <button data-act="view" data-id="${esc(r.id)}" class="btn ${highlightRoute === r.id ? 'btn-amber' : 'btn-soft'} !px-2" title="Xem đường đi trên bản đồ"><i data-lucide="eye" class="w-4 h-4"></i></button>
                    <button data-act="edit" data-id="${esc(r.id)}" class="btn btn-soft !px-2" title="Sửa" ${running === r.id ? 'disabled' : ''}><i data-lucide="pencil" class="w-4 h-4"></i></button>
                    <button data-act="del" data-id="${esc(r.id)}" class="btn btn-danger !px-2" title="Xóa" ${running === r.id ? 'disabled' : ''}><i data-lucide="trash-2" class="w-4 h-4"></i></button>
                </div>
            </div>
            <div class="flex flex-wrap items-center gap-1.5 text-xs">${stepChips(r)}</div>
        </div>`).join('')
        : `<div class="col-span-full text-center text-slate-500 border-2 border-dashed border-slate-300 rounded-xl p-10">${S && S.selected_map ? (WP.length ? 'Chưa có lộ trình nào. Bấm <b>"Tạo lộ trình"</b> rồi chọn các điểm theo thứ tự.' : 'Bản đồ này chưa có điểm nào - hãy tạo vài điểm ở tab <b>Điểm đến</b> trước.') : 'Hãy chọn hoặc quét một bản đồ trước.'}</div>`;

    // ---- trang chính: danh sách gọn
    $('routes-panel').innerHTML = ROUTES.length ? ROUTES.map((r) => `
        <div class="flex items-center gap-2 p-2 rounded-lg border ${highlightRoute === r.id || running === r.id ? 'border-purple-300 bg-purple-50' : 'border-slate-200 bg-slate-50'}">
            <button data-act="view" data-id="${esc(r.id)}" class="flex-1 text-left min-w-0" title="Bấm để xem đường đi trên bản đồ">
                <div class="text-sm font-semibold text-slate-900 truncate">${esc(r.name)}</div>
                <div class="text-[11px] text-slate-500 truncate">${r.steps.length} điểm${r.loop ? ' · ↻ lặp' : ''} · ${esc(routePts(r.steps).map((w) => w.name).join(' → '))}</div>
            </button>
            <button data-act="run" data-id="${esc(r.id)}" class="btn btn-primary !px-3 !py-1.5" ${ok ? '' : 'disabled'} title="${esc(hint)}"><i data-lucide="play" class="w-4 h-4"></i>Chạy</button>
        </div>`).join('') + (ok || running ? '' : `<p class="text-[11px] text-amber-700 bg-amber-50 border border-amber-200 rounded-lg p-2">${esc(hint)}</p>`)
        : `<p class="text-xs text-slate-500">${S && S.selected_map ? 'Chưa có lộ trình. Vào tab <b>Lộ trình</b> để tạo từ các điểm có sẵn.' : 'Chưa chọn bản đồ.'}</p>`;
    icons();
}
function routeClick(e) {
    const b = e.target.closest('button[data-act]');
    if (!b) return;
    const id = b.dataset.id;
    if (b.dataset.act === 'run') runRoute(id);
    else if (b.dataset.act === 'view') { highlightRoute = highlightRoute === id ? null : id; routesKey = ''; renderRoutes(); }
    else if (b.dataset.act === 'edit') editRoute(id);
    else if (b.dataset.act === 'del') deleteRoute(id);
}
$('routes-list').addEventListener('click', routeClick);
$('routes-panel').addEventListener('click', routeClick);

async function runRoute(id) {
    const r = routeById(id);
    if (!r) return;
    if (await act(() => api(`/api/routes/${id}/run`, 'POST'), `Bắt đầu lộ trình "${r.name}"`)) { highlightRoute = null; switchTab('dashboard'); }
}
async function stopRoute() {
    if (!(await confirmBox('Dừng lộ trình?', 'Robot sẽ dừng lại tại chỗ và các điểm còn lại bị hủy.', 'Dừng lộ trình', true))) return;
    await act(() => api('/api/routes/stop', 'POST'), 'Đã dừng lộ trình');
}
async function deleteRoute(id) {
    const r = routeById(id);
    if (!r || !(await confirmBox('Xóa lộ trình?', `Xóa "${esc(r.name)}"? (các điểm vẫn được giữ)`, 'Xóa', true))) return;
    if (await act(() => api(`/api/maps/${encodeURIComponent(S.selected_map)}/routes/${id}`, 'DELETE'), 'Đã xóa lộ trình')) reloadRoutes();
}
async function confirmStep(taskId) { await act(() => api(`/api/tasks/${taskId}/confirm`, 'POST'), 'Đã xác nhận giao xong'); }
async function togglePause() {
    const paused = !(S.run && S.run.paused);
    await act(() => api('/api/tasks/pause', 'POST', { paused }), paused ? 'Đã tạm dừng: robot dừng sau bước hiện tại' : 'Lộ trình tiếp tục');
}
function saveTaskSettings() {
    act(() => api('/api/settings', 'POST', { require_confirm: $('chk-confirm').checked, return_home: $('chk-home').checked }));
}

// ---- thẻ trạng thái đang chạy
let runKey = '';
function stepStatusChip(t) { return `<span class="chip ${TASK_CHIP[t.status]} ${t.status === 'going' ? 'animate-pulse' : ''}">${TASK_LABEL[t.status]}</span>`; }
function runCardHtml(full) {
    const r = S.run;
    const steps = S.tasks.filter((t) => t.route_step);
    if (!r && !(full && steps.length)) return '';
    let head = '', actions = '';
    if (r) {
        const cur = r.current;
        const pct = r.total ? (r.done / r.total) * 100 : 0;
        head = `
            <div class="flex flex-wrap items-center justify-between gap-2">
                <div class="min-w-0"><div class="text-[11px] font-bold text-purple-700 uppercase">Đang chạy lộ trình${r.loop ? ` · vòng ${r.cycle}` : ''}${r.paused ? ' · TẠM DỪNG' : ''}</div>
                <div class="font-bold text-slate-900 truncate">${esc(r.route_name)}</div></div>
                <div class="text-xs font-mono text-slate-600">${r.done}/${r.total} điểm</div>
            </div>
            <div class="w-full bg-slate-200 rounded-full h-1.5"><div class="bg-purple-600 h-1.5 rounded-full transition-all" style="width:${pct}%"></div></div>
            <div class="text-xs text-slate-700">${cur ? `Bước ${cur.step}/${r.total} → <b>${esc(cur.name)}</b> (${TASK_LABEL[cur.status]})` : (r.paused ? 'Đang tạm dừng' : 'Đang chuẩn bị bước tiếp theo...')}</div>`;
        actions = `<div class="flex flex-wrap gap-2">
            ${cur && cur.status === 'arrived' ? `<button data-act="confirm" data-id="${esc(cur.id)}" class="btn btn-primary flex-1"><i data-lucide="check" class="w-4 h-4"></i>Đã giao - đi tiếp</button>` : ''}
            <button data-act="pause" class="btn btn-soft">${r.paused ? 'Tiếp tục' : 'Tạm dừng'}</button>
            <button data-act="stop" class="btn btn-danger"><i data-lucide="square" class="w-4 h-4"></i>Dừng</button></div>`;
    } else {
        head = `<div class="flex justify-between items-center"><div class="text-[11px] font-bold text-slate-500 uppercase">Lượt chạy gần nhất</div><button data-act="clear" class="text-xs text-blue-600 hover:underline font-semibold">Xóa</button></div>`;
    }
    const list = full ? `<div class="space-y-1.5">${steps.map((t) => `<div class="flex items-center justify-between gap-2 text-sm bg-slate-50 rounded-lg px-3 py-1.5"><span class="truncate"><b class="text-slate-400 font-mono mr-2">${t.route_step}</b>${esc(t.name)}</span>${stepStatusChip(t)}</div>`).join('')}</div>` : '';
    return `<div class="bg-white ${r ? 'border-2 border-purple-300' : 'border border-slate-200'} rounded-xl p-3 shadow-sm space-y-2.5">${head}${actions}${list}</div>`;
}
function renderRun() {
    const cc = $('chk-confirm'), ch = $('chk-home');
    if (document.activeElement !== cc) cc.checked = S.settings.require_confirm;
    if (document.activeElement !== ch) ch.checked = S.settings.return_home;
    const key = JSON.stringify([S.run, S.tasks]);
    if (key === runKey) return;
    runKey = key;
    $('run-card-dash').innerHTML = runCardHtml(false);
    $('run-card-tab').innerHTML = runCardHtml(true);
    icons();
}
function runClick(e) {
    const b = e.target.closest('button[data-act]');
    if (!b) return;
    if (b.dataset.act === 'confirm') confirmStep(b.dataset.id);
    else if (b.dataset.act === 'pause') togglePause();
    else if (b.dataset.act === 'stop') stopRoute();
    else if (b.dataset.act === 'clear') act(() => api('/api/tasks/clear', 'POST', { finished_only: true }));
}
$('run-card-dash').addEventListener('click', runClick);
$('run-card-tab').addEventListener('click', runClick);

function announceRunEnd(prev) {
    if (!prev || !prev.run || S.run) return;
    const name = prev.run.route_name;
    const st = S.tasks.filter((t) => t.route_step).map((t) => t.status);
    if (st.includes('failed')) toast(`Lộ trình "${name}" dừng: robot không tới được một điểm`, 'error');
    else if (st.includes('cancelled')) toast(`Đã dừng lộ trình "${name}"`);
    else toast(`Hoàn thành lộ trình "${name}"`, 'ok');
}

// ---- trình tạo / sửa lộ trình
function newRoute() {
    if (!S.selected_map) { toast('Hãy chọn hoặc quét một bản đồ trước', 'error'); return; }
    if (!WP.length) { toast('Bản đồ này chưa có điểm nào - tạo điểm ở tab "Điểm đến" trước', 'error'); return; }
    editor = { id: null, name: '', loop: false, steps: [] };
    openEditor();
}
function editRoute(id) {
    const r = routeById(id);
    if (!r) return;
    editor = { id: r.id, name: r.name, loop: r.loop, steps: r.steps.filter((sid) => WP.find((w) => w.id === sid)) };
    openEditor();
}
function openEditor() {
    $('route-editor').classList.remove('hidden');
    $('re-name').value = editor.name;
    $('re-loop').checked = editor.loop;
    renderEditor();
    $('route-editor').scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    $('re-name').focus();
}
function cancelRouteEdit() { editor = null; $('route-editor').classList.add('hidden'); }
function renderEditor() {
    if (!editor) return;
    $('re-available').innerHTML = WP.map((w) => `
        <div class="flex items-center gap-2 p-2 rounded-lg border border-slate-200 bg-slate-50">
            <span class="w-2.5 h-2.5 rounded-full shrink-0" style="background:${WP_COLOR[w.type] || '#64748b'}"></span>
            <div class="flex-1 min-w-0"><div class="text-sm font-semibold truncate">${esc(w.name)}</div><div class="text-[11px] text-slate-500">${esc(WP_TYPE[w.type] || w.type)}</div></div>
            <button data-add="${esc(w.id)}" class="btn btn-primary !px-2.5 !py-1" title="Thêm vào cuối lộ trình"><i data-lucide="plus" class="w-4 h-4"></i></button>
        </div>`).join('');
    $('re-steps').innerHTML = editor.steps.length ? editor.steps.map((id, i) => {
        const w = WP.find((x) => x.id === id);
        return `<div draggable="true" data-i="${i}" class="flex items-center gap-2 p-2 rounded-lg border border-purple-200 bg-purple-50 cursor-grab">
            <span class="w-6 h-6 rounded-full bg-purple-600 text-white text-xs font-bold flex items-center justify-center shrink-0">${i + 1}</span>
            <div class="flex-1 min-w-0 text-sm font-semibold truncate">${w ? esc(w.name) : '(đã xóa)'}</div>
            <button data-mv="-1" data-i="${i}" class="btn btn-soft !px-2 !py-1" ${i === 0 ? 'disabled' : ''} title="Lên">▲</button>
            <button data-mv="1" data-i="${i}" class="btn btn-soft !px-2 !py-1" ${i === editor.steps.length - 1 ? 'disabled' : ''} title="Xuống">▼</button>
            <button data-rm="${i}" class="btn btn-danger !px-2 !py-1" title="Bỏ khỏi lộ trình"><i data-lucide="x" class="w-4 h-4"></i></button>
        </div>`;
    }).join('') : `<div class="text-xs text-slate-500 border-2 border-dashed border-slate-300 rounded-lg p-4 text-center">Chưa có điểm nào. Bấm <b>+</b> ở danh sách bên trái theo thứ tự robot cần đi.</div>`;
    icons();
}
$('re-available').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-add]');
    if (b && editor) { editor.steps.push(b.dataset.add); renderEditor(); }
});
$('re-loop').addEventListener('change', () => { if (editor) editor.loop = $('re-loop').checked; });
$('re-steps').addEventListener('click', (e) => {
    if (!editor) return;
    const mv = e.target.closest('button[data-mv]'), rm = e.target.closest('button[data-rm]');
    if (mv) moveStep(+mv.dataset.i, +mv.dataset.i + +mv.dataset.mv);
    else if (rm) { editor.steps.splice(+rm.dataset.rm, 1); renderEditor(); }
});
function moveStep(from, to) {
    if (!editor || to < 0 || to >= editor.steps.length || from === to) return;
    const [x] = editor.steps.splice(from, 1);
    editor.steps.splice(to, 0, x);
    renderEditor();
}
let dragFrom = null;
$('re-steps').addEventListener('dragstart', (e) => { const it = e.target.closest('[data-i]'); if (it) { dragFrom = +it.dataset.i; e.dataTransfer.effectAllowed = 'move'; e.dataTransfer.setData('text/plain', String(dragFrom)); } });
$('re-steps').addEventListener('dragover', (e) => { if (dragFrom !== null) e.preventDefault(); });
$('re-steps').addEventListener('drop', (e) => {
    e.preventDefault();
    const it = e.target.closest('[data-i]');
    if (dragFrom !== null && it) moveStep(dragFrom, +it.dataset.i);
    dragFrom = null;
});
$('re-steps').addEventListener('dragend', () => { dragFrom = null; });

async function saveRoute() {
    if (!editor) return;
    const name = $('re-name').value.trim();
    if (!name) { toast('Hãy đặt tên cho lộ trình', 'error'); $('re-name').focus(); return; }
    if (!editor.steps.length) { toast('Lộ trình cần ít nhất 1 điểm', 'error'); return; }
    const body = { name, steps: editor.steps, loop: $('re-loop').checked };
    const base = `/api/maps/${encodeURIComponent(S.selected_map)}/routes`;
    const r = await act(() => (editor.id ? api(`${base}/${editor.id}`, 'PUT', body) : api(base, 'POST', body)), `Đã lưu lộ trình "${name}"`);
    if (r) { cancelRouteEdit(); await reloadRoutes(); }
}

// ============================================================ tab Hệ thống
const openLogs = new Set();
let stacksKey = '';
function renderStacks(force) {
    if (!S) return;
    const key = JSON.stringify(S.stacks);
    if (!force && key === stacksKey) return;
    stacksKey = key;
    const chip = { stopped: 'chip-gray', starting: 'chip-amber', running: 'chip-green', external: 'chip-blue', stopping: 'chip-amber', failed: 'chip-red' };
    $('stacks-grid').innerHTML = Object.entries(S.stacks).map(([name, st]) => {
        const activeSt = ['starting', 'running', 'external'].includes(st.state);
        const startBtn = name === 'nav'
            ? `<button class="btn btn-soft" disabled title="Bật Nav2 bằng nút Điều hướng ở tab Bản đồ">Bật ở tab Bản đồ</button>`
            : `<button data-act="start" data-name="${name}" class="btn btn-primary" ${activeSt || st.state === 'stopping' ? 'disabled' : ''}><i data-lucide="play" class="w-4 h-4"></i>Bật</button>`;
        const stopBtn = `<button data-act="stop" data-name="${name}" class="btn btn-danger" ${!st.managed && st.state !== 'starting' && st.state !== 'running' ? 'disabled' : ''}><i data-lucide="square" class="w-4 h-4"></i>Tắt</button>`;
        return `
        <div class="bg-white border border-slate-200 rounded-xl p-4 shadow-sm space-y-3">
            <div class="flex justify-between items-start gap-2">
                <div><h3 class="font-bold text-sm text-slate-900">${esc(STACK_TITLE[name] || name)}</h3><p class="text-[11px] font-mono text-slate-500 break-all mt-1">${esc(st.cmd)}</p></div>
                <span class="chip ${chip[st.state]} shrink-0">${STACK_LABEL[st.state]}${st.state === 'running' || st.state === 'external' ? '' : ''}</span>
            </div>
            <div class="text-xs text-slate-500">${st.ready ? '<span class="text-emerald-600 font-semibold">Node đã sẵn sàng</span>' : 'Chưa thấy node đặc trưng'}${st.pid ? ` · pid ${st.pid}` : ''}${st.exit_code !== null && st.state === 'failed' ? ` · mã thoát ${st.exit_code}` : ''}</div>
            <div class="flex gap-2">${startBtn}${stopBtn}<button data-act="log" data-name="${name}" class="btn btn-soft ml-auto"><i data-lucide="terminal" class="w-4 h-4"></i>Log</button></div>
            <pre id="log-${name}" class="${openLogs.has(name) ? '' : 'hidden'} bg-slate-900 text-slate-100 text-[11px] leading-snug p-3 rounded-lg overflow-auto max-h-64 whitespace-pre-wrap break-all"></pre>
        </div>`;
    }).join('');
    icons();
    openLogs.forEach(pollLog);
}
$('stacks-grid').addEventListener('click', async (e) => {
    const b = e.target.closest('button[data-act]');
    if (!b) return;
    const name = b.dataset.name;
    if (b.dataset.act === 'log') { openLogs.has(name) ? openLogs.delete(name) : openLogs.add(name); $('log-' + name).classList.toggle('hidden', !openLogs.has(name)); if (openLogs.has(name)) pollLog(name); }
    else if (b.dataset.act === 'start') act(() => api(`/api/stacks/${name}/start`, 'POST'));
    else if (b.dataset.act === 'stop') {
        const extra = name === 'bringup' ? 'Robot sẽ ngừng nhận lệnh và mất dữ liệu cảm biến.' : 'Robot sẽ quay về chế độ IDLE.';
        if (await confirmBox(`Tắt ${STACK_TITLE[name] || name}?`, extra, 'Tắt', true)) act(() => api(`/api/stacks/${name}/stop`, 'POST'));
    }
});
async function pollLog(name) {
    const pre = $('log-' + name);
    if (!pre || !openLogs.has(name)) return;
    try {
        const r = await api(`/api/stacks/${name}/log?n=150`);
        const atEnd = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 30;
        pre.textContent = r.lines.join('\n') || '(chưa có log)';
        if (atEnd) pre.scrollTop = pre.scrollHeight;
    } catch { /* bỏ qua */ }
}
setInterval(() => { if (activeTab === 'system') openLogs.forEach(pollLog); }, 2000);
setInterval(() => { if (activeTab === 'maps') refreshMaps(); }, 6000);

function setController(v) { act(() => api('/api/settings', 'POST', { controller: v }), `Controller = ${v} (áp dụng ở lần bật Nav2 kế tiếp)`); }

// ============================================================ điều hướng / E-STOP
async function cancelNav() { await act(() => api('/api/nav/cancel', 'POST'), 'Đã gửi yêu cầu hủy'); }
async function toggleEstop() {
    const on = !(S && S.estop);
    stopTeleop();
    await act(() => api('/api/estop', 'POST', { on }), on ? 'ĐÃ BẬT E-STOP: robot dừng, mọi lệnh bị chặn' : 'Đã nhả E-STOP');
}

// ============================================================ canvas bản đồ
const canvas = $('mapCanvas');
const ctx = canvas.getContext('2d');
const view = { s: 40, ox: 0, oy: 0, key: undefined, fitW: 0 }; // key undefined = chưa căn khung
let cw = 0, ch = 0, dpr = 1;
const w2s = (x, y) => [view.ox + x * view.s, view.oy - y * view.s];
const s2w = (sx, sy) => [(sx - view.ox) / view.s, (view.oy - sy) / view.s];

function resizeCanvas() {
    const r = canvas.parentElement.getBoundingClientRect();
    dpr = window.devicePixelRatio || 1;
    cw = Math.max(1, r.width); ch = Math.max(1, r.height);
    canvas.width = Math.round(cw * dpr); canvas.height = Math.round(ch * dpr);
    if (view.fitW < 100 && cw >= 100) view.key = undefined;
}
new ResizeObserver(resizeCanvas).observe(canvas.parentElement);
window.addEventListener('resize', resizeCanvas);

const imgCache = new Map();
function getImage(url) {
    let e = imgCache.get(url);
    if (!e) {
        const im = new Image();
        e = { im, ok: false };
        im.onload = () => { e.ok = true; };
        im.src = url;
        imgCache.set(url, e);
        if (imgCache.size > 10) imgCache.delete(imgCache.keys().next().value);
    }
    return e.ok ? e.im : null;
}

function currentMap() {
    if (!S) return null;
    if (S.mode === 'mapping' && S.live_map) {
        const m = S.live_map;
        return { key: 'live', url: `/api/live_map.png?v=${m.version}`, res: m.resolution, ox: m.origin_x, oy: m.origin_y, w: m.width, h: m.height };
    }
    const info = S.selected_map && mapsInfo[S.selected_map];
    if (info) return { key: info.name + ':' + info.mtime, url: `/api/maps/${encodeURIComponent(info.name)}/image.png?v=${info.mtime}`, res: info.resolution, ox: info.origin[0], oy: info.origin[1], w: info.width, h: info.height };
    return null;
}

function fitView() {
    const m = currentMap();
    view.fitW = cw;
    if (!m) {
        const p = S && S.pose.valid ? S.pose : { x: 0, y: 0 };
        view.s = 40; view.ox = cw / 2 - p.x * view.s; view.oy = ch / 2 + p.y * view.s;
        view.key = null;
        return;
    }
    const W = m.w * m.res, H = m.h * m.res;
    view.s = clamp(Math.min((cw - 60) / W, (ch - 60) / H), 5, 400);
    view.ox = cw / 2 - (m.ox + W / 2) * view.s;
    view.oy = ch / 2 + (m.oy + H / 2) * view.s;
    view.key = m.key.split(':')[0];
}
function zoomBy(f, sx = cw / 2, sy = ch / 2) {
    const [wx, wy] = s2w(sx, sy);
    view.s = clamp(view.s * f, 5, 400);
    view.ox = sx - wx * view.s; view.oy = sy + wy * view.s;
}
function toggleLayer(name) {
    layers[name] = !layers[name];
    setClass($('btn-' + name), `p-2 rounded-lg border border-transparent ${layers[name] ? 'toggle-on' : 'hover:bg-slate-100 text-slate-700'}`);
}

function arrow(x, y, theta, len, color, width = 2.5) { // (x,y) toạ độ màn hình, theta hệ thế giới
    const ex = x + Math.cos(theta) * len, ey = y - Math.sin(theta) * len;
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(ex, ey); ctx.stroke();
    const a = Math.atan2(-(ey - y), ex - x), h = Math.max(6, len * 0.28);
    ctx.beginPath();
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - h * Math.cos(a - 0.45), ey + h * Math.sin(a - 0.45));
    ctx.lineTo(ex - h * Math.cos(a + 0.45), ey + h * Math.sin(a + 0.45));
    ctx.closePath(); ctx.fill();
}

function drawGrid() {
    const steps = [0.1, 0.5, 1, 2, 5, 10, 20, 50];
    const step = steps.find((s) => s * view.s >= 45) || 50;
    const [x0, y1] = s2w(0, 0), [x1, y0] = s2w(cw, ch);
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#e2e8f0';
    ctx.beginPath();
    for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) { const [sx] = w2s(x, 0); ctx.moveTo(Math.round(sx) + 0.5, 0); ctx.lineTo(Math.round(sx) + 0.5, ch); }
    for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) { const [, sy] = w2s(0, y); ctx.moveTo(0, Math.round(sy) + 0.5); ctx.lineTo(cw, Math.round(sy) + 0.5); }
    ctx.stroke();
    ctx.strokeStyle = '#94a3b8';
    const [ox, oy] = w2s(0, 0);
    ctx.beginPath(); ctx.moveTo(ox, 0); ctx.lineTo(ox, ch); ctx.moveTo(0, oy); ctx.lineTo(cw, oy); ctx.stroke();
    ctx.fillStyle = '#64748b'; ctx.font = '10px ui-monospace,monospace';
    ctx.fillText(`${step} m/ô`, 8, ch - 8);
}

function drawRoutePath() {
    let steps = null, loop = false, running = false;
    if (S.run) { const r = routeById(S.run.route_id); if (r) { steps = r.steps; loop = r.loop; running = true; } }
    if (!steps && editor) { steps = editor.steps; loop = editor.loop; }
    if (!steps && highlightRoute) { const r = routeById(highlightRoute); if (r) { steps = r.steps; loop = r.loop; } }
    if (!steps || !steps.length) return;
    const P = steps.map((id) => { const w = WP.find((x) => x.id === id); return w ? w2s(w.x, w.y) : null; });
    const status = {};
    if (running) S.tasks.forEach((t) => { if (t.route_step) status[t.route_step] = t.status; });

    const seg = (a, b, alpha) => {
        if (!a || !b) return;
        ctx.strokeStyle = `rgba(124,58,237,${alpha})`; ctx.fillStyle = `rgba(124,58,237,${alpha})`;
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
        const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2, ang = Math.atan2(b[1] - a[1], b[0] - a[0]);
        if (Math.hypot(b[0] - a[0], b[1] - a[1]) > 40) {   // mũi tên chỉ chiều ở giữa đoạn
            ctx.save(); ctx.setLineDash([]); ctx.translate(mx, my); ctx.rotate(ang);
            ctx.beginPath(); ctx.moveTo(7, 0); ctx.lineTo(-5, -5); ctx.lineTo(-5, 5); ctx.closePath(); ctx.fill(); ctx.restore();
        }
    };
    ctx.save(); ctx.lineWidth = 3; ctx.setLineDash([9, 6]);
    for (let i = 0; i + 1 < P.length; i++) seg(P[i], P[i + 1], 0.85);
    if (loop && P.length > 2) seg(P[P.length - 1], P[0], 0.35);
    ctx.restore();

    ctx.save(); ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.font = 'bold 10px Inter, system-ui, sans-serif';
    steps.forEach((id, i) => {
        if (!P[i]) return;
        const k = steps.slice(0, i).filter((x) => x === id).length;
        const bx = P[i][0] + 15 + 18 * k, by = P[i][1] - 15;
        const st = status[i + 1];
        ctx.fillStyle = st === 'done' ? '#16a34a' : (st === 'going' || st === 'arrived') ? '#2563eb' : '#7c3aed';
        ctx.strokeStyle = '#fff'; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(bx, by, 9, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        ctx.fillStyle = '#fff'; ctx.fillText(String(i + 1), bx, by + 0.5);
    });
    ctx.restore();
}

function drawWaypoints() {
    ctx.textAlign = 'center';
    for (const w of WP) {
        const [sx, sy] = w2s(w.x, w.y);
        if (sx < -30 || sy < -30 || sx > cw + 30 || sy > ch + 30) continue;
        const col = WP_COLOR[w.type] || '#64748b';
        const sel = selectedWp === w.id;
        ctx.save(); ctx.translate(sx, sy);
        ctx.fillStyle = col; ctx.strokeStyle = sel ? '#0f172a' : '#fff'; ctx.lineWidth = sel ? 3 : 2;
        ctx.beginPath();
        if (w.type === 'DOCK') ctx.rect(-8, -8, 16, 16);
        else if (w.type === 'TABLE') ctx.roundRect(-9, -9, 18, 18, 5);
        else ctx.arc(0, 0, 9, 0, Math.PI * 2);
        ctx.fill(); ctx.stroke(); ctx.restore();
        arrow(sx, sy, w.theta, 20, col, 2);
        ctx.fillStyle = '#0f172a'; ctx.font = '600 11px Inter, system-ui, sans-serif';
        ctx.strokeStyle = 'rgba(255,255,255,.9)'; ctx.lineWidth = 3;
        ctx.strokeText(w.name, sx, sy + 26); ctx.fillText(w.name, sx, sy + 26);
    }
    ctx.textAlign = 'start';
}

function drawRobot() {
    const p = S.pose;
    const [sx, sy] = w2s(p.x, p.y);
    let L = S.robot.length * view.s, W = S.robot.width * view.s;
    const k = Math.max(1, 22 / Math.max(L, W)); L *= k; W *= k; // luôn đủ to để nhìn thấy khi thu nhỏ
    ctx.save(); ctx.translate(sx, sy); ctx.rotate(-p.theta);
    ctx.fillStyle = 'rgba(37,99,235,.22)'; ctx.strokeStyle = '#2563eb'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.rect(-L / 2, -W / 2, L, W); ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#2563eb';
    ctx.beginPath(); ctx.moveTo(L / 2 - 1, 0); ctx.lineTo(L / 2 - Math.min(L * 0.4, 14), -Math.min(W * 0.28, 8)); ctx.lineTo(L / 2 - Math.min(L * 0.4, 14), Math.min(W * 0.28, 8)); ctx.closePath(); ctx.fill();
    ctx.beginPath(); ctx.arc(0, 0, 3, 0, Math.PI * 2); ctx.fill();
    ctx.restore();
}

function drawFrame() {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = '#f1f5f9'; ctx.fillRect(0, 0, cw, ch);
    if (!S) { ctx.fillStyle = '#64748b'; ctx.font = '14px Inter, system-ui'; ctx.fillText('Đang kết nối tới robot...', 20, 30); return; }

    const m = currentMap();
    if (view.key !== (m ? m.key.split(':')[0] : null)) fitView();

    if (layers.grid) drawGrid();
    if (m) {
        const img = getImage(m.url);
        if (img) {
            const [x, y] = w2s(m.ox, m.oy + m.h * m.res);
            ctx.imageSmoothingEnabled = false;
            ctx.drawImage(img, x, y, m.w * m.res * view.s, m.h * m.res * view.s);
        }
    }

    const showPose = S.pose.valid && (S.pose.frame === 'map' || !m);
    if (S.plan.length > 1 && (S.pose.frame === 'map')) {
        ctx.strokeStyle = '#2563eb'; ctx.lineWidth = 2.5; ctx.setLineDash([8, 6]);
        ctx.beginPath(); S.plan.forEach(([x, y], i) => { const [sx, sy] = w2s(x, y); i ? ctx.lineTo(sx, sy) : ctx.moveTo(sx, sy); }); ctx.stroke();
        ctx.setLineDash([]);
    }
    if (layers.scan && showPose && S.scan.points.length && S.scan.frame === S.pose.frame) {
        ctx.fillStyle = '#f43f5e';
        for (const [x, y] of S.scan.points) { const [sx, sy] = w2s(x, y); ctx.fillRect(sx - 1.2, sy - 1.2, 2.6, 2.6); }
    }
    drawRoutePath();
    drawWaypoints();

    if (navBusy() && S.nav.goal && S.pose.frame === 'map') {
        const g = S.nav.goal, [gx, gy] = w2s(g.x, g.y);
        ctx.strokeStyle = '#2563eb'; ctx.fillStyle = 'rgba(37,99,235,.18)'; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(gx, gy, 13, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        arrow(gx, gy, g.theta, 26, '#2563eb');
    }
    if (showPose) drawRobot();

    if (drag && drag.mode === 'pose') { // xem trước: điểm + hướng đang kéo
        const col = { goal: '#2563eb', init: '#16a34a', add: '#f59e0b' }[tool];
        const [px, py] = w2s(drag.wx, drag.wy);
        ctx.strokeStyle = col; ctx.fillStyle = col + '33'; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(px, py, 12, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        arrow(px, py, drag.theta, 34, col, 3);
    }
}
function frame() { try { drawFrame(); } catch (e) { console.error('draw', e); } requestAnimationFrame(frame); }

// ------------------------------------------------------------ công cụ + con trỏ
const TOOL_HINT = {
    goal: 'Nhấn vào điểm đến (nhấn giữ & kéo để chọn hướng)', init: 'Nhấn-kéo tại chỗ robot đang đứng để đặt vị trí + hướng ban đầu',
    add: 'Nhấn-kéo để thêm điểm (kéo để chọn hướng)',
};
const TOOL_ON = 'px-2.5 py-1.5 font-bold text-xs rounded-lg flex items-center gap-1.5 transition tool-on';
const TOOL_OFF = 'px-2.5 py-1.5 font-bold text-xs rounded-lg flex items-center gap-1.5 transition text-slate-700 hover:bg-slate-100';
function setTool(t) {
    tool = t;
    ['pan', 'goal', 'init', 'add'].forEach((k) => setClass($('tool-' + k), k === t ? TOOL_ON : TOOL_OFF));
    const hint = $('tool-hint');
    hint.classList.toggle('hidden', t === 'pan');
    hint.textContent = TOOL_HINT[t] || '';
    canvas.parentElement.style.cursor = t === 'pan' ? 'grab' : 'crosshair';
}

let drag = null;
function pointerPos(e) { const r = canvas.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
canvas.addEventListener('contextmenu', (e) => e.preventDefault());
canvas.addEventListener('pointerdown', (e) => {
    canvas.setPointerCapture(e.pointerId);
    const [sx, sy] = pointerPos(e);
    const [wx, wy] = s2w(sx, sy);
    if (e.button === 1 || e.button === 2 || tool === 'pan') drag = { mode: 'pan', sx, sy, ox0: view.ox, oy0: view.oy, moved: false, button: e.button };
    else drag = { mode: 'pose', sx, sy, wx, wy, theta: null, moved: false };
    e.preventDefault();
});
canvas.addEventListener('pointermove', (e) => {
    if (!drag) return;
    const [sx, sy] = pointerPos(e);
    const dx = sx - drag.sx, dy = sy - drag.sy;
    if (Math.hypot(dx, dy) > 5) drag.moved = true;
    if (drag.mode === 'pan') { if (drag.moved) { view.ox = drag.ox0 + dx; view.oy = drag.oy0 + dy; canvas.parentElement.style.cursor = 'grabbing'; } }
    else if (Math.hypot(dx, dy) > 10) drag.theta = Math.atan2(-dy, dx);
});
canvas.addEventListener('pointerup', (e) => {
    if (!drag) return;
    const d = drag; drag = null;
    canvas.parentElement.style.cursor = tool === 'pan' ? 'grab' : 'crosshair';
    if (d.mode === 'pan') {
        if (!d.moved && d.button === 0) hitTestWaypoint(d.sx, d.sy);
        return;
    }
    finishPose(d);
});
canvas.addEventListener('pointercancel', () => { drag = null; });
canvas.addEventListener('wheel', (e) => { e.preventDefault(); const [sx, sy] = pointerPos(e); zoomBy(e.deltaY < 0 ? 1.15 : 1 / 1.15, sx, sy); }, { passive: false });

function hitTestWaypoint(sx, sy) {
    let best = null, bd = 18;
    for (const w of WP) { const [px, py] = w2s(w.x, w.y); const d = Math.hypot(px - sx, py - sy); if (d < bd) { bd = d; best = w; } }
    selectedWp = best ? best.id : null;
    renderWpCard(); renderWaypoints();
}

async function finishPose(d) {
    const { wx, wy } = d;
    if (tool === 'goal') {
        if (!mapFrameOk()) { toast('Chưa có vị trí robot trên bản đồ - vào chế độ Điều hướng và đặt vị trí ban đầu trước', 'error'); return; }
        const theta = d.theta ?? Math.atan2(wy - S.pose.y, wx - S.pose.x);
        if (await act(() => api('/api/nav/goal', 'POST', { x: wx, y: wy, theta, name: null }), 'Đã gửi điểm đến')) navTrack.d0 = null;
    } else if (tool === 'init') {
        if (S.mode !== 'navigation') { toast('Chỉ đặt vị trí ban đầu được ở chế độ Điều hướng (AMCL)', 'error'); return; }
        await act(() => api('/api/initialpose', 'POST', { x: wx, y: wy, theta: d.theta ?? 0 }), 'Đã gửi vị trí ban đầu - chờ AMCL hội tụ');
        setTool('pan');
    } else if (tool === 'add') {
        await createWaypoint(wx, wy, d.theta ?? 0);
        setTool('pan');
    }
}

function renderWpCard() {
    const card = $('wp-card');
    const w = selectedWp && WP.find((x) => x.id === selectedWp);
    if (!w) { card.classList.add('hidden'); return; }
    card.classList.remove('hidden');
    card.innerHTML = `
        <div class="flex justify-between items-start"><div><div class="font-bold text-sm text-slate-900">${esc(w.name)}</div><div class="text-[11px] text-slate-500 font-mono">${esc(WP_TYPE[w.type] || w.type)} · ${fmt(w.x)}, ${fmt(w.y)} m · ${fmt(deg(w.theta), 0)}°</div></div>
        <button data-act="close" class="text-slate-400 hover:text-slate-700"><i data-lucide="x" class="w-4 h-4"></i></button></div>
        <div class="flex flex-wrap gap-1.5">
            <button data-act="go" class="btn btn-primary !py-1 flex-1" ${navReady() && !S.estop ? '' : 'disabled'}>Đi tới</button>
            <button data-act="edit" class="btn btn-soft !py-1"><i data-lucide="pencil" class="w-3.5 h-3.5"></i></button>
            <button data-act="del" class="btn btn-danger !py-1"><i data-lucide="trash-2" class="w-3.5 h-3.5"></i></button>
        </div>`;
    icons();
}
$('wp-card').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-act]');
    if (!b || !selectedWp) return;
    if (b.dataset.act === 'close') { selectedWp = null; renderWpCard(); renderWaypoints(); } else wpAction(b.dataset.act, selectedWp);
});

// ============================================================ teleop
const teleop = { lin: 0, ang: 0, timer: null, keys: new Set() };
const teleopCfg = { lin: parseFloat(store.get('lin', 0.3)), ang: parseFloat(store.get('ang', 0.8)) };
function sendTeleop() { if (ws && ws.readyState === 1) ws.send(JSON.stringify({ type: 'teleop', linear: teleop.lin, angular: teleop.ang })); }
function setTeleop(lin, ang) {
    if (S && S.estop) return;
    teleop.lin = lin; teleop.ang = ang;
    if (!teleop.timer) teleop.timer = setInterval(sendTeleop, 100);
    sendTeleop();
}
function stopTeleopLocal() { clearInterval(teleop.timer); teleop.timer = null; teleop.lin = teleop.ang = 0; teleop.keys.clear(); }
function stopTeleop() { const was = teleop.timer !== null; stopTeleopLocal(); if (was) sendTeleop(); }
function syncTeleopSliders() {
    $('rng-lin').value = teleopCfg.lin; $('rng-ang').value = teleopCfg.ang;
    teleopCfg.lin = parseFloat($('rng-lin').value); teleopCfg.ang = parseFloat($('rng-ang').value); // bị ép theo min/max mới
    setText('lbl-lin', `${teleopCfg.lin.toFixed(2)} m/s`); setText('lbl-ang', `${teleopCfg.ang.toFixed(2)} rad/s`);
}
$('rng-lin').addEventListener('input', () => { teleopCfg.lin = parseFloat($('rng-lin').value); store.set('lin', teleopCfg.lin); syncTeleopSliders(); });
$('rng-ang').addEventListener('input', () => { teleopCfg.ang = parseFloat($('rng-ang').value); store.set('ang', teleopCfg.ang); syncTeleopSliders(); });

const DIRS = { up: [1, 0], down: [-1, 0], left: [0, 1], right: [0, -1] };
document.querySelectorAll('.teleop-btn').forEach((b) => {
    const [fx, fz] = DIRS[b.dataset.dir];
    b.addEventListener('pointerdown', (e) => { b.setPointerCapture(e.pointerId); setTeleop(fx * teleopCfg.lin, fz * teleopCfg.ang); e.preventDefault(); });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((ev) => b.addEventListener(ev, stopTeleop));
});
const KEYS = { w: 'up', arrowup: 'up', s: 'down', arrowdown: 'down', a: 'left', arrowleft: 'left', d: 'right', arrowright: 'right' };
function keyTeleop() {
    let fx = 0, fz = 0;
    teleop.keys.forEach((d) => { fx += DIRS[d][0]; fz += DIRS[d][1]; });
    if (!fx && !fz) stopTeleop(); else setTeleop(fx * teleopCfg.lin, fz * teleopCfg.ang);
}
window.addEventListener('keydown', (e) => {
    if (modalOpen() || activeTab !== 'dashboard' || /^(INPUT|SELECT|TEXTAREA)$/.test(document.activeElement.tagName)) return;
    const k = e.key.toLowerCase();
    if (k === ' ') { stopTeleop(); e.preventDefault(); return; }
    if (!(k in KEYS) || e.repeat) return;
    teleop.keys.add(KEYS[k]); keyTeleop(); e.preventDefault();
});
window.addEventListener('keyup', (e) => { const k = e.key.toLowerCase(); if (k in KEYS) { teleop.keys.delete(KEYS[k]); keyTeleop(); } });
window.addEventListener('blur', stopTeleop);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopTeleop(); });
window.addEventListener('beforeunload', stopTeleop);

// ============================================================ khởi động
resizeCanvas();
syncTeleopSliders();
setTool('pan');
switchTab(['dashboard', 'maps', 'waypoints', 'routes', 'system'].includes(store.get('tab', '')) ? store.get('tab') : 'dashboard');
refreshMaps();
connectWS();
requestAnimationFrame(frame);
icons();
