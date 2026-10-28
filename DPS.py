"""
hotspot_kalman_locator.py - Hệ thống Định vị 2D Thời Gian Thực (60 FPS Continuous DSP)
Phiên bản: True-Angle 2D Positioning + Positive Quadrant Enforcement (X >= 0, Y >= 0)

ĐÃ KHẮC PHỤC TRIỆT ĐỂ LỖI GÓC RADAR & TỌA ĐỘ ÂM:
1. Xóa bỏ hoàn toàn thuật toán "auto_2d_orbit" tự bịa góc xoay từ nhiễu RSSI.
2. Đưa toàn bộ góc mặc định của 3 điện thoại về Góc Phần Tư Dương (20°, 45°, 70° -> X >= 0, Y >= 0),
   xóa bỏ góc âm (-35°) và góc tù (145°) từng làm điện thoại bị tụt xuống dưới trục ngang của Radar.
3. Bổ sung 3 Chế độ Không gian Thực (Mặc định: Khóa Tọa Độ Dương X >= 0, Y >= 0).
4. Bổ sung chế độ Giao hội 2 Điểm Neo (2D Trilateration) và Chuẩn hóa Góc Chính Diện PC (Tare 0°)
   trên Web Beacon để hiển thị đúng góc thực tế của từng máy.
"""

import collections
import ctypes
from ctypes import wintypes
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
import socket
import threading
import time
from typing import Dict, List, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import numpy as np


# ============================================================================
# PHẦN 1: CẤU TRÚC WINDOWS NATIVE WIFI API (WLANAPI.DLL 64-BIT)
# ============================================================================
class GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", ctypes.c_ubyte * 8),
    ]


class WLAN_INTERFACE_INFO(ctypes.Structure):
    _fields_ = [
        ("InterfaceGuid", GUID),
        ("strInterfaceDescription", ctypes.c_wchar * 256),
        ("isState", ctypes.c_uint),
    ]


class WLAN_INTERFACE_INFO_LIST(ctypes.Structure):
    _fields_ = [
        ("dwNumberOfItems", wintypes.DWORD),
        ("dwIndex", wintypes.DWORD),
        ("InterfaceInfo", WLAN_INTERFACE_INFO * 8),
    ]


class DOT11_SSID(ctypes.Structure):
    _fields_ = [
        ("uSSIDLength", ctypes.c_ulong),
        ("ucSSID", ctypes.c_ubyte * 32),
    ]


class WLAN_RATE_SET(ctypes.Structure):
    _fields_ = [
        ("uRateSetLength", ctypes.c_ulong),
        ("usRateSet", ctypes.c_ushort * 126),
    ]


class WLAN_BSS_ENTRY(ctypes.Structure):
    _fields_ = [
        ("dot11Ssid", DOT11_SSID),
        ("uPhyId", ctypes.c_ulong),
        ("dot11Bssid", ctypes.c_ubyte * 6),
        ("dot11BssType", ctypes.c_uint),
        ("uDot11BssPhyType", ctypes.c_uint),
        ("lRssi", ctypes.c_long),
        ("uLinkQuality", ctypes.c_ulong),
        ("bInRegDomain", ctypes.c_ubyte),
        ("usBeaconPeriod", ctypes.c_ushort),
        ("ullTimestamp", ctypes.c_ulonglong),
        ("ullHostTimestamp", ctypes.c_ulonglong),
        ("usCapabilityInformation", ctypes.c_ushort),
        ("ulChCenterFrequency", ctypes.c_ulong),
        ("wlanRateSet", WLAN_RATE_SET),
        ("ulIeOffset", ctypes.c_ulong),
        ("ulIeSize", ctypes.c_ulong),
    ]


# ============================================================================
# PHẦN 2: BỘ CHỐNG VỌT BẤT ĐỐI XỨNG, TỰ ĐỘNG HIỆU CHUẨN & KALMAN HỘI TỤ NHANH
# ============================================================================
class AsymmetricMultipathFilter:
    """Bộ lọc chống vọt khoảng cách bất đối xứng (chỉ đẩy mẫu mới thực sự vào cửa sổ Hampel)."""
    def __init__(self, max_drop_db: float = 4.5, max_rise_db: float = 8.0, window_size: int = 9):
        self.max_drop = float(max_drop_db)
        self.max_rise = float(max_rise_db)
        self.window = collections.deque(maxlen=window_size)
        self.ref: Optional[float] = None
        self.last_raw_pushed: Optional[float] = None
        self.last_push_ts: float = 0.0
        self.drop_hold_time: float = 0.0

    def filter(self, z: float, dt: float, is_fast_lock: bool = False) -> Tuple[float, bool]:
        z = float(z)
        now = time.perf_counter()

        if self.ref is None or is_fast_lock:
            self.ref = z
            self.window.append(z)
            self.last_raw_pushed = z
            self.last_push_ts = now
            self.drop_hold_time = 0.0
            return z, False

        if self.last_raw_pushed is None or abs(z - self.last_raw_pushed) > 0.05 or (now - self.last_push_ts) >= 0.12:
            self.window.append(z)
            self.last_raw_pushed = z
            self.last_push_ts = now

        arr = np.array(self.window, dtype=float)
        med = float(np.median(arr))
        mad = float(np.median(np.abs(arr - med))) + 0.25
        sigma_est = 1.4826 * mad

        diff = z - self.ref

        if diff < -self.max_drop:
            self.drop_hold_time += dt
            if self.drop_hold_time < 0.65 and z < med - 1.8 * sigma_est:
                return self.ref, True

            slew_rate = 3.5 + 6.0 * min(2.0, self.drop_hold_time)
            self.ref = max(z, self.ref - slew_rate * dt)
            return self.ref, (self.drop_hold_time < 0.45)

        elif diff > self.max_rise:
            self.drop_hold_time = 0.0
            slew_rise = 12.0 * dt
            self.ref = min(z, self.ref + slew_rise)
            return self.ref, True

        self.drop_hold_time = max(0.0, self.drop_hold_time - dt * 2.0)
        target = 0.75 * z + 0.25 * med
        alpha = min(1.0, dt * 14.0)
        self.ref += alpha * (target - self.ref)
        return self.ref, False

    def reset(self):
        self.ref = None
        self.window.clear()
        self.last_raw_pushed = None
        self.drop_hold_time = 0.0


class HandHeldMotionDiscriminator:
    """Cổng phát hiện chuyển động nhạy bén với Fast-Lock."""
    def __init__(self, min_move_dist_m: float = 0.35, min_move_dbm: float = 2.2):
        self.min_move_dist_m = float(min_move_dist_m)
        self.min_move_dbm = float(min_move_dbm)

        self.anchor_rssi: Optional[float] = None
        self.anchor_dist: Optional[float] = None
        self.is_moving: bool = False
        self.move_accum_time: float = 0.0
        self.static_accum_time: float = 0.0
        self.last_sign: int = 0
        self.recent_rssi = collections.deque(maxlen=20)

    def set_min_thresholds(self, min_dist_m: float, min_dbm: float):
        self.min_move_dist_m = max(0.15, float(min_dist_m))
        self.min_move_dbm = max(1.0, float(min_dbm))

    def evaluate(
        self, z_rssi: float, raw_dist: float, current_kf_dist: float, dt: float, is_fast_lock: bool = False
    ) -> Tuple[bool, float, float, str]:
        self.recent_rssi.append(z_rssi)
        if self.anchor_rssi is None or self.anchor_dist is None or is_fast_lock:
            self.anchor_rssi = z_rssi
            self.anchor_dist = raw_dist
            self.is_moving = True if is_fast_lock else False
            return self.is_moving, 0.0, self.min_move_dist_m, "KHÓA VỊ TRÍ TỨC THÌ (Fast-Lock)"

        ref_d = max(0.8, self.anchor_dist)
        eff_min_dist = self.min_move_dist_m + 0.10 * max(0.0, ref_d - 1.2)
        eff_min_dbm = self.min_move_dbm + 0.20 * max(0.0, ref_d - 1.5)

        delta_rssi = z_rssi - self.anchor_rssi
        delta_dist = abs(raw_dist - self.anchor_dist)
        sign = 1 if delta_rssi >= 0 else -1

        asym_factor = 1.15 if delta_rssi < 0 else 0.95
        exceeds_minimum = (delta_dist >= eff_min_dist * asym_factor) and (abs(delta_rssi) >= eff_min_dbm * asym_factor)

        if not self.is_moving:
            if exceeds_minimum:
                speed_boost = 2.0 if abs(delta_rssi) >= eff_min_dbm * 1.8 else 1.0
                if sign == self.last_sign:
                    self.move_accum_time += dt * speed_boost
                else:
                    self.move_accum_time = dt * 0.6
                self.last_sign = sign

                if self.move_accum_time >= 0.18:
                    self.is_moving = True
                    self.static_accum_time = 0.0
                    return True, delta_dist, eff_min_dist, f"ĐANG DI CHUYỂN (Δ={delta_dist:.2f}m)"
                else:
                    return False, delta_dist, eff_min_dist, f"XÁC NHẬN CHUYỂN ĐỘNG ({self.move_accum_time:.2f}/0.18s)"
            else:
                self.move_accum_time = max(0.0, self.move_accum_time - dt * 2.0)
                alpha = min(0.25, 1.2 * dt)
                self.anchor_rssi = (1.0 - alpha) * self.anchor_rssi + alpha * z_rssi
                self.anchor_dist = (1.0 - alpha) * self.anchor_dist + alpha * current_kf_dist
                return False, delta_dist, eff_min_dist, f"ĐỨNG YÊN (Ổn định ±{delta_dist:.2f}m)"
        else:
            recent_span = (max(self.recent_rssi) - min(self.recent_rssi)) if len(self.recent_rssi) >= 6 else 99.0
            if recent_span < (eff_min_dbm * 0.80) and abs(raw_dist - current_kf_dist) < 0.25:
                self.static_accum_time += dt
            else:
                self.static_accum_time = 0.0

            if self.static_accum_time >= 0.40:
                self.is_moving = False
                self.move_accum_time = 0.0
                self.anchor_rssi = z_rssi
                self.anchor_dist = current_kf_dist
                return False, delta_dist, eff_min_dist, "ĐỨNG YÊN (Đã khóa vị trí chuẩn)"

            alpha_m = min(0.50, 3.0 * dt)
            self.anchor_rssi = (1.0 - alpha_m) * self.anchor_rssi + alpha_m * z_rssi
            self.anchor_dist = current_kf_dist
            return True, delta_dist, eff_min_dist, f"ĐANG DI CHUYỂN (Δ={delta_dist:.2f}m)"

    def reset(self, rssi_val: Optional[float] = None, dist_val: Optional[float] = None):
        self.anchor_rssi = rssi_val
        self.anchor_dist = dist_val
        self.is_moving = False
        self.move_accum_time = 0.0
        self.static_accum_time = 0.0
        self.recent_rssi.clear()


class ContinuousRSSIKalman:
    """Bộ lọc Kalman Tầng 1 với chế độ hội tụ tức thì (Fast-Lock)."""
    def __init__(self, q_static: float = 0.04, q_moving: float = 2.8, r_meas: float = 1.8):
        self.q_static = float(q_static)
        self.q_moving = float(q_moving)
        self.r_meas = float(r_meas)
        self.x: Optional[float] = None
        self.p: float = 1.0
        self.k_gain: float = 0.0

    def update(self, z_dbm: float, is_moving: bool, dt: float, is_fast_lock: bool = False) -> Tuple[float, float]:
        z = float(z_dbm)
        dt = max(0.005, min(0.5, float(dt)))
        if self.x is None or is_fast_lock:
            self.x = z
            self.p = 0.5
            self.k_gain = 1.0
            return self.x, self.k_gain

        innovation = z - self.x
        asym_r_mult = 2.0 if innovation < -2.5 else 1.0

        if is_moving:
            q_eff = self.q_moving * dt
            r_eff = self.r_meas * asym_r_mult * 0.75
        else:
            q_eff = self.q_static * dt
            r_eff = self.r_meas * 2.2 * asym_r_mult
            innovation = max(-1.2, min(1.2, innovation)) * 0.45

        p_pred = self.p + q_eff
        self.k_gain = p_pred / (p_pred + r_eff)
        self.x = self.x + self.k_gain * innovation
        self.p = (1.0 - self.k_gain) * p_pred
        return self.x, self.k_gain

    def reset(self, val: Optional[float] = None):
        self.x = float(val) if val is not None else None
        self.p = 1.0


class ContinuousKinematicKalman:
    """Bộ lọc Kalman Động học Tầng 2."""
    def __init__(self, sigma_accel: float = 0.85, sigma_meas: float = 0.22):
        self.sigma_a = float(sigma_accel)
        self.sigma_z = float(sigma_meas)
        self.x: Optional[np.ndarray] = None
        self.P: np.ndarray = np.eye(2)

    def update(self, d_meas: float, is_moving: bool, dt: float, is_fast_lock: bool = False) -> Tuple[float, float]:
        z = float(d_meas)
        dt = max(0.005, min(0.5, float(dt)))
        if self.x is None or is_fast_lock:
            self.x = np.array([[z], [0.0]], dtype=float)
            self.P = np.diag([0.05, 0.02])
            return z, 0.0

        prev_d = float(self.x[0, 0])
        gamma = math.pow(0.55, dt) if is_moving else 0.0

        F = np.array([
            [1.0, dt],
            [0.0, gamma]
        ], dtype=float)
        H = np.array([[1.0, 0.0]], dtype=float)

        q_scale = (self.sigma_a ** 2) if is_moving else (self.sigma_a * 0.08) ** 2
        G = np.array([[0.5 * dt ** 2], [dt]], dtype=float)
        Q = (G @ G.T) * q_scale

        dist_factor = max(1.0, prev_d / 1.8)
        outward_jump = max(0.0, z - prev_d)
        spike_penalty = 1.0 + 1.8 * (outward_jump ** 2)

        r_base = self.sigma_z * dist_factor * (0.85 if is_moving else 2.0)
        R = np.array([[(r_base ** 2) * spike_penalty]], dtype=float)

        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q

        y = np.array([[z]]) - (H @ x_pred)
        S = H @ P_pred @ H.T + R
        K = P_pred @ H.T @ np.linalg.inv(S)

        self.x = x_pred + K @ y
        self.P = (np.eye(2) - K @ H) @ P_pred

        d_est = float(self.x[0, 0])
        v_est = float(self.x[1, 0])

        if is_moving:
            max_outward_step = 2.2 * dt
            max_inward_step = 3.0 * dt
            delta = d_est - prev_d
            if delta > max_outward_step:
                d_est = prev_d + max_outward_step
                v_est = min(v_est, 2.2)
            elif delta < -max_inward_step:
                d_est = prev_d - max_inward_step
                v_est = max(v_est, -3.0)
        else:
            v_est = 0.0
            max_static_settle = 0.15 * dt
            if abs(d_est - prev_d) > max_static_settle:
                d_est = prev_d + math.copysign(max_static_settle, d_est - prev_d)

        d_est = float(np.clip(d_est, 0.05, 15.0))
        v_est = float(np.clip(v_est, -3.0, 2.2))
        self.x[0, 0] = d_est
        self.x[1, 0] = v_est
        return d_est, v_est

    def reset(self, d_val: Optional[float] = None):
        if d_val is None:
            self.x = None
        else:
            self.x = np.array([[float(d_val)], [0.0]], dtype=float)
            self.P = np.diag([0.05, 0.02])


class AutoCalibratingLogDistanceModel:
    """Mô hình tự động hiệu chuẩn theo băng tần 2.4GHz / 5GHz."""
    def __init__(self, default_a_24g: float = -43.5, default_n: float = 2.25):
        self.base_a_24g = float(default_a_24g)
        self.a = float(default_a_24g)
        self.n = float(default_n)
        self.current_freq_mhz: int = 2412
        self.stable_rssi_samples: collections.deque = collections.deque(maxlen=120)
        self.initialized_band: bool = False

    def update_auto_calibration(self, clean_rssi: float, freq_mhz: int, is_moving: bool):
        if freq_mhz > 0 and (not self.initialized_band or abs(freq_mhz - self.current_freq_mhz) > 500):
            self.current_freq_mhz = int(freq_mhz)
            self.initialized_band = True
            if freq_mhz >= 4900:
                self.a = self.base_a_24g - 6.2
                self.n = 2.35
            else:
                self.a = self.base_a_24g
                self.n = 2.20

        if not is_moving and -85.0 <= clean_rssi <= -22.0:
            self.stable_rssi_samples.append(clean_rssi)
            if len(self.stable_rssi_samples) >= 20:
                arr = np.array(self.stable_rssi_samples, dtype=float)
                std_val = float(np.std(arr))
                p90_rssi = float(np.percentile(arr, 90))

                if p90_rssi > self.a + 8.0:
                    target_a = p90_rssi - 6.0
                    self.a = 0.96 * self.a + 0.04 * target_a

                if std_val < 1.2:
                    target_n = 2.15 if self.current_freq_mhz < 4900 else 2.28
                elif std_val > 2.8:
                    target_n = 2.45
                else:
                    target_n = 2.25
                self.n = 0.98 * self.n + 0.02 * target_n

    def rssi_to_distance(self, rssi_dbm: float) -> float:
        exponent = (self.a - float(rssi_dbm)) / (10.0 * self.n)
        d = math.pow(10.0, exponent)
        return max(0.05, min(15.0, d))


# ============================================================================
# PHẦN 3: BỘ THU THẬP TÍN HIỆU & TRẠM ĐỒNG BỘ GÓC THỰC TẾ (CÓ NÚT CHUẨN HÓA 0°)
# ============================================================================
WEB_BEACON_HTML = """<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hotspot 2D True-Angle Beacon</title>
<style>
  body { background:#0b0f19; color:#00ffcc; font-family:sans-serif; text-align:center; padding:18px; }
  .box { background:#161f33; border:1px solid #2a3b5c; border-radius:12px; padding:16px; margin:12px auto; max-width:390px; }
  button, select { background:#0284c7; color:#fff; border:none; padding:10px 16px; border-radius:8px; font-size:15px; font-weight:bold; margin:6px; cursor:pointer; }
  .tare { background:#059669; }
</style>
</head>
<body>
  <h2>ĐỒNG BỘ GÓC THỰC 2D (TRUE AZIMUTH)</h2>
  <div class="box">
    <label>Chọn Thiết bị:</label>
    <select id="dev">
      <option value="0">Điện thoại 1</option>
      <option value="1">Điện thoại 2</option>
      <option value="2">Điện thoại 3</option>
    </select><br>
    <button onclick="enableCompass()">1. Bật Cảm Biến Góc</button>
    <button class="tare" onclick="tareZero()">2. Đặt Hướng Hiện Tại = 0° (Chính diện PC)</button>
    <p id="st">Hướng điện thoại về phía PC rồi bấm nút số 2 để chuẩn hóa góc!</p>
    <h3 id="ang">Góc thực gửi về PC: 0.0°</h3>
  </div>
<script>
let rawHeading = 0.0;
let zeroOffset = 0.0;
function enableCompass() {
  if (typeof DeviceOrientationEvent !== 'undefined' && typeof DeviceOrientationEvent.requestPermission === 'function') {
    DeviceOrientationEvent.requestPermission().then(r => { if (r==='granted') window.addEventListener('deviceorientation', onDir); });
  } else {
    window.addEventListener('deviceorientation', onDir);
  }
  document.getElementById('st').innerText = 'Đã bật cảm biến góc xoay 2D!';
}
function tareZero() {
  zeroOffset = rawHeading;
  document.getElementById('st').innerText = 'Đã khóa mốc 0° chính diện PC!';
}
function onDir(e) {
  if (e.webkitCompassHeading !== undefined) rawHeading = e.webkitCompassHeading;
  else if (e.alpha !== null) rawHeading = (360 - e.alpha) % 360;
}
setInterval(() => {
  const d = document.getElementById('dev').value;
  let relAngle = (rawHeading - zeroOffset + 360.0) % 360.0;
  document.getElementById('ang').innerText = `Góc thực gửi về PC: ${relAngle.toFixed(1)}°`;
  fetch(`/telemetry?dev=${d}&heading=${relAngle.toFixed(1)}`).catch(()=>{});
}, 80);
</script>
</body>
</html>
"""

FRESH_SEC = 20.0
STALE_SEC = 45.0

# Góc mặc định của 3 điện thoại đều nằm trong Góc Phần Tư Dương (X >= 0, Y >= 0):
# 20°, 45°, 70° -> sin > 0 và cos > 0 tuyệt đối!
DEFAULT_POSITIVE_ANGLES = {0: 20.0, 1: 45.0, 2: 70.0}


class HighSpeedSignalCollector:
    """Thu thập mẫu Wi-Fi RSSI tốc độ cao với tọa độ góc mặc định nằm hoàn toàn ở miền dương."""
    def __init__(self, udp_port: int = 5005, http_port: int = 8080, start_threads: bool = True):
        self.udp_port = udp_port
        self.http_port = http_port
        self._lock = threading.Lock()
        self._stop = False
        self.start_time = time.time()

        self.bss_db: Dict[str, Dict[str, object]] = {}
        self.ssid_rssi_windows: Dict[str, collections.deque] = {}
        self.pinned_bssid_per_ssid: Dict[str, str] = {}
        self.cached_networks: Dict[str, Dict[str, object]] = {}

        # Khởi tạo góc trong Góc Phần Tư Thứ Nhất (X >= 0, Y >= 0), không còn góc -35° hay 145°
        self.device_headings: Dict[int, float] = dict(DEFAULT_POSITIVE_ANGLES)
        self.has_live_compass: Dict[int, bool] = {0: False, 1: False, 2: False}

        self._wlanapi = self._init_wlanapi()
        self._wlan_handle = wintypes.HANDLE()
        if self._wlanapi is not None:
            ver = wintypes.DWORD()
            if self._wlanapi.WlanOpenHandle(2, None, ctypes.byref(ver), ctypes.byref(self._wlan_handle)) != 0:
                self._wlan_handle = wintypes.HANDLE()

        initial_map = self._read_hardware_once(trigger_scan=True)
        with self._lock:
            if initial_map:
                self._merge_scan(initial_map, time.time())
                self.cached_networks = self._build_networks_by_ssid(time.time())

        if start_threads:
            threading.Thread(target=self._fast_hardware_worker, daemon=True).start()
            threading.Thread(target=self._udp_worker, daemon=True).start()
            threading.Thread(target=self._http_worker, daemon=True).start()

    def stop(self):
        self._stop = True
        if self._wlanapi is not None and self._wlan_handle.value:
            try:
                self._wlanapi.WlanCloseHandle(self._wlan_handle, None)
            except Exception:
                pass
            self._wlan_handle = wintypes.HANDLE()

    @staticmethod
    def _init_wlanapi():
        try:
            w = ctypes.windll.wlanapi
            w.WlanOpenHandle.argtypes = [wintypes.DWORD, ctypes.c_void_p, ctypes.POINTER(wintypes.DWORD), ctypes.POINTER(wintypes.HANDLE)]
            w.WlanOpenHandle.restype = wintypes.DWORD
            w.WlanEnumInterfaces.argtypes = [wintypes.HANDLE, ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(WLAN_INTERFACE_INFO_LIST))]
            w.WlanEnumInterfaces.restype = wintypes.DWORD
            w.WlanScan.argtypes = [wintypes.HANDLE, ctypes.POINTER(GUID), ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
            w.WlanScan.restype = wintypes.DWORD
            w.WlanQueryInterface.argtypes = [
                wintypes.HANDLE, ctypes.POINTER(GUID), ctypes.c_uint, ctypes.c_void_p,
                ctypes.POINTER(wintypes.DWORD), ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_uint)
            ]
            w.WlanQueryInterface.restype = wintypes.DWORD
            w.WlanGetNetworkBssList.argtypes = [
                wintypes.HANDLE, ctypes.POINTER(GUID), ctypes.c_void_p, ctypes.c_uint,
                wintypes.BOOL, ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)
            ]
            w.WlanGetNetworkBssList.restype = wintypes.DWORD
            w.WlanFreeMemory.argtypes = [ctypes.c_void_p]
            w.WlanFreeMemory.restype = None
            w.WlanCloseHandle.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
            w.WlanCloseHandle.restype = wintypes.DWORD
            return w
        except Exception:
            return None

    def _read_hardware_once(self, trigger_scan: bool = False) -> Dict[str, Dict[str, object]]:
        if self._wlanapi is None or not self._wlan_handle.value:
            return {}

        bss_map: Dict[str, Dict[str, object]] = {}
        p_if_list = ctypes.POINTER(WLAN_INTERFACE_INFO_LIST)()
        if self._wlanapi.WlanEnumInterfaces(self._wlan_handle, None, ctypes.byref(p_if_list)) == 0 and bool(p_if_list):
            try:
                num_ifs = min(8, int(p_if_list.contents.dwNumberOfItems))
                for i in range(num_ifs):
                    guid = p_if_list.contents.InterfaceInfo[i].InterfaceGuid
                    if trigger_scan:
                        self._wlanapi.WlanScan(self._wlan_handle, ctypes.byref(guid), None, None, None)

                    p_bss = ctypes.c_void_p()
                    rc_bss = self._wlanapi.WlanGetNetworkBssList(
                        self._wlan_handle, ctypes.byref(guid), None, 3, False, None, ctypes.byref(p_bss)
                    )
                    if rc_bss == 0 and p_bss.value:
                        try:
                            base = p_bss.value
                            count = min(128, max(0, int(ctypes.c_uint32.from_address(base + 4).value)))
                            sz = ctypes.sizeof(WLAN_BSS_ENTRY)
                            for j in range(count):
                                entry = WLAN_BSS_ENTRY.from_address(base + 8 + j * sz)
                                slen = min(32, int(entry.dot11Ssid.uSSIDLength))
                                if slen <= 0:
                                    continue
                                ssid_s = bytes(entry.dot11Ssid.ucSSID[:slen]).decode("utf-8", errors="ignore").strip()
                                if not ssid_s:
                                    continue
                                rssi_v = float(entry.lRssi)
                                if not (-100 <= rssi_v <= -10):
                                    continue
                                freq_v = int(entry.ulChCenterFrequency // 1000)
                                bssid_s = ":".join(f"{b:02x}" for b in entry.dot11Bssid)
                                host_ts = int(entry.ullHostTimestamp)

                                prev = bss_map.get(bssid_s)
                                if prev is None or host_ts >= int(prev["host_ts"]):
                                    bss_map[bssid_s] = {
                                        "ssid": ssid_s,
                                        "bssid": bssid_s,
                                        "rssi": rssi_v,
                                        "freq_mhz": freq_v,
                                        "host_ts": host_ts,
                                    }
                        finally:
                            self._wlanapi.WlanFreeMemory(p_bss)
            finally:
                self._wlanapi.WlanFreeMemory(ctypes.cast(p_if_list, ctypes.c_void_p))

        return bss_map

    def _merge_scan(self, bss_map: Dict[str, Dict[str, object]], now: float):
        for bssid, new in bss_map.items():
            old = self.bss_db.get(bssid)
            if old is None:
                entry_copy = dict(new)
                entry_copy["first_seen"] = now
                entry_copy["last_update"] = now
                self.bss_db[bssid] = entry_copy
                continue

            ts_changed = int(new["host_ts"]) != int(old["host_ts"])
            rssi_changed = abs(float(new["rssi"]) - float(old["rssi"])) > 0.1
            if ts_changed or rssi_changed:
                old["last_update"] = now
            old["ssid"] = new["ssid"]
            old["rssi"] = new["rssi"]
            old["freq_mhz"] = new["freq_mhz"]
            old["host_ts"] = new["host_ts"]

        for b in [b for b, e in self.bss_db.items()
                  if b not in bss_map and (now - float(e["last_update"])) > STALE_SEC]:
            del self.bss_db[b]

    def _build_networks_by_ssid(self, now: float) -> Dict[str, Dict[str, object]]:
        groups: Dict[str, List[Dict[str, object]]] = {}
        for e in self.bss_db.values():
            if (now - float(e["last_update"])) <= FRESH_SEC:
                groups.setdefault(str(e["ssid"]), []).append(e)

        if not groups and self.bss_db:
            for e in self.bss_db.values():
                groups.setdefault(str(e["ssid"]), []).append(e)

        out: Dict[str, Dict[str, object]] = {}
        for ssid, lst in groups.items():
            pinned_mac = self.pinned_bssid_per_ssid.get(ssid)
            chosen_entry = None
            if pinned_mac:
                for e in lst:
                    if e["bssid"] == pinned_mac and (now - float(e["last_update"])) <= 8.0:
                        chosen_entry = e
                        break

            if chosen_entry is None:
                newest_host_ts = max(int(e["host_ts"]) for e in lst)
                fresh_candidates = [e for e in lst if (newest_host_ts - int(e["host_ts"])) <= 30_000_000]
                pool = fresh_candidates if fresh_candidates else lst
                chosen_entry = max(pool, key=lambda x: float(x["rssi"]))
                self.pinned_bssid_per_ssid[ssid] = str(chosen_entry["bssid"])

            # Tìm thêm BSSID phụ (nếu phát đồng thời 2.4GHz/5GHz hoặc có điểm tham chiếu)
            secondary_rssi = None
            for e in lst:
                if e["bssid"] != chosen_entry["bssid"]:
                    secondary_rssi = float(e["rssi"])
                    break

            win = self.ssid_rssi_windows.setdefault(ssid, collections.deque(maxlen=5))
            raw_r = float(chosen_entry["rssi"])
            if not win or abs(raw_r - win[-1]) > 0.05 or len(win) < 3:
                win.append(raw_r)

            w_sorted = sorted(win)
            if len(w_sorted) >= 3:
                clean_rssi = float(0.5 * np.mean(w_sorted) + 0.5 * np.median(w_sorted))
            else:
                clean_rssi = raw_r

            out[ssid] = {
                "ssid": ssid,
                "bssid": chosen_entry["bssid"],
                "rssi": clean_rssi,
                "raw_rssi": raw_r,
                "secondary_rssi": secondary_rssi,
                "freq_mhz": int(chosen_entry["freq_mhz"]),
                "count": len(lst),
            }
        return out

    def _fast_hardware_worker(self):
        last_scan_time = time.time()
        while not self._stop:
            now = time.time()
            should_scan = (now - last_scan_time) >= 1.10
            if should_scan:
                last_scan_time = now

            bss_map = self._read_hardware_once(trigger_scan=should_scan)
            with self._lock:
                if bss_map:
                    self._merge_scan(bss_map, now)
                self.cached_networks = self._build_networks_by_ssid(now)

            time.sleep(0.04)

    def _udp_worker(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", self.udp_port))
            sock.settimeout(0.2)
            while not self._stop:
                try:
                    data, _ = sock.recvfrom(1024)
                    payload = json.loads(data.decode("utf-8", errors="ignore"))
                    with self._lock:
                        dev_idx = int(payload.get("dev", 0))
                        if "heading_deg" in payload and dev_idx in self.device_headings:
                            self.device_headings[dev_idx] = float(payload["heading_deg"])
                            self.has_live_compass[dev_idx] = True
                except Exception:
                    continue
        finally:
            sock.close()

    def _http_worker(self):
        collector = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/telemetry":
                    qs = parse_qs(parsed.query)
                    with collector._lock:
                        dev_idx = int(qs.get("dev", ["0"])[0])
                        if "heading" in qs and qs["heading"][0] and dev_idx in collector.device_headings:
                            try:
                                collector.device_headings[dev_idx] = float(qs["heading"][0])
                                collector.has_live_compass[dev_idx] = True
                            except ValueError:
                                pass
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"ok":true}')
                else:
                    body = WEB_BEACON_HTML.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)

            def log_message(self, format, *args):
                return

        try:
            server = HTTPServer(("0.0.0.0", self.http_port), Handler)
            server.timeout = 0.5
            while not self._stop:
                server.handle_request()
        except Exception:
            pass

    def set_manual_azimuth(self, dev_idx: int, angle_deg: float):
        with self._lock:
            self.device_headings[dev_idx] = float(angle_deg)

    def get_realtime_snapshot(self) -> Dict[str, object]:
        with self._lock:
            nets = dict(self.cached_networks)
            headings = dict(self.device_headings)
            live_compass = dict(self.has_live_compass)
        return {
            "timestamp": time.time(),
            "networks": nets,
            "headings": headings,
            "live_compass": live_compass,
        }


# ============================================================================
# PHẦN 4: BỘ ĐỊNH VỊ 2D CHUẨN XÁC (KHÔNG QUỸ ĐẠO ẢO & ĐẢM BẢO TỌA ĐỘ DƯƠNG)
# ============================================================================
class Realtime2DHotspotLocator:
    """
    Bộ định vị 60 FPS với 3 Chế độ Không gian Thực (Workspace Mode):
    - "POSITIVE_XY": Khóa chặt trong Góc phần tư thứ nhất (X >= 0, Y >= 0, góc [0°, 90°]).
                     Tuyệt đối không bao giờ có tọa độ âm hay tụt xuống dưới trục ngang.
    - "FRONT_Y":     Bán cầu phía trước màn hình (Y >= 0, góc [-90°, 90°]).
    - "FULL_360":    Toàn mặt phẳng 360° (chỉ dùng khi kết nối cảm biến góc thực từ điện thoại).
    """
    def __init__(self, dev_index: int = 0):
        self.dev_index = dev_index
        self.spike = AsymmetricMultipathFilter(max_drop_db=4.5, max_rise_db=8.0, window_size=9)
        self.motion_gate = HandHeldMotionDiscriminator(min_move_dist_m=0.35, min_move_dbm=2.2)
        self.rssi_kf = ContinuousRSSIKalman(q_static=0.04, q_moving=2.8, r_meas=1.8)
        self.model = AutoCalibratingLogDistanceModel(default_a_24g=-43.5, default_n=2.25)
        self.dist_kf = ContinuousKinematicKalman(sigma_accel=0.85, sigma_meas=0.22)

        self.max_len = 240
        self.hist_raw_rssi = collections.deque(maxlen=self.max_len)
        self.hist_kf_rssi = collections.deque(maxlen=self.max_len)
        self.hist_raw_dist = collections.deque(maxlen=self.max_len)
        self.hist_kf_dist = collections.deque(maxlen=self.max_len)
        self.hist_xy = collections.deque(maxlen=90)

        self.last_kf_rssi: Optional[float] = None
        self.last_kf_dist: Optional[float] = None
        self.smooth_display_dist: Optional[float] = None
        self.last_step_time: Optional[float] = None
        self.lock_start_time: Optional[float] = None
        self.step_count: int = 0

        # Góc khởi tạo chuẩn trong Góc Phần Tư Dương: 20°, 45°, 70° (X >= 0, Y >= 0)
        self.azimuth_deg: float = DEFAULT_POSITIVE_ANGLES.get(dev_index % 3, 45.0)
        self.workspace_mode: str = "POSITIVE_XY"  # Mặc định: X >= 0, Y >= 0
        self.use_trilateration: bool = False
        self.anchor_baseline_m: float = 1.5       # Khoảng cách mốc neo phụ trên bàn (X_anchor, 0)

        self.smooth_x: Optional[float] = None
        self.smooth_y: Optional[float] = None

    @staticmethod
    def classify_zone(d: float) -> Tuple[str, str]:
        if d < 0.8:
            return "VÙNG 1: Sát máy tính (0 - 0.8m)", "#00ff88"
        if d < 2.0:
            return "VÙNG 2: Quanh bàn làm việc (0.8 - 2.0m)", "#00d4ff"
        if d < 4.5:
            return "VÙNG 3: Trong phòng (2.0 - 4.5m)", "#ffcc00"
        return "VÙNG 4: Xa máy tính (> 4.5m)", "#ff5555"

    def clamp_angle_by_workspace(self, angle_deg: float) -> float:
        """Đảm bảo góc luôn nằm trong vùng không gian vật lý đã chọn."""
        if self.workspace_mode == "POSITIVE_XY":
            # Khóa chặt góc trong [0°, 90°] -> sin(rad) >= 0 (X >= 0) và cos(rad) >= 0 (Y >= 0)
            norm = ((angle_deg + 180.0) % 360.0) - 180.0
            return float(np.clip(abs(norm), 2.0, 88.0))
        elif self.workspace_mode == "FRONT_Y":
            # Khóa góc trong [-88°, +88°] -> cos(rad) >= 0 (Y >= 0, luôn ở phía trên trục ngang)
            norm = ((angle_deg + 180.0) % 360.0) - 180.0
            return float(np.clip(norm, -88.0, 88.0))
        return float(angle_deg % 360.0)

    def soft_reset(self):
        self.spike.reset()
        self.rssi_kf.reset()
        self.dist_kf.reset()
        self.motion_gate.reset()
        self.hist_raw_rssi.clear()
        self.hist_kf_rssi.clear()
        self.hist_raw_dist.clear()
        self.hist_kf_dist.clear()
        self.hist_xy.clear()
        self.last_kf_rssi = None
        self.last_kf_dist = None
        self.smooth_display_dist = None
        self.smooth_x = None
        self.smooth_y = None
        self.last_step_time = None
        self.lock_start_time = None
        self.step_count = 0

    def step_realtime(
        self,
        raw_rssi_in: float,
        freq_mhz: int,
        user_heading_deg: float,
        has_live_compass: bool,
        secondary_rssi: Optional[float] = None,
    ) -> Dict[str, object]:
        now = time.perf_counter()
        if self.last_step_time is None:
            dt = 1.0 / 60.0
        else:
            dt = max(0.004, min(0.25, now - self.last_step_time))
        self.last_step_time = now

        if self.lock_start_time is None:
            self.lock_start_time = now

        elapsed_since_lock = now - self.lock_start_time
        is_fast_lock = (self.step_count < 3) or (elapsed_since_lock <= 0.45)
        self.step_count += 1

        # 0. Tự động hiệu chuẩn mô hình suy hao theo băng tần 2.4GHz / 5GHz
        self.model.update_auto_calibration(
            clean_rssi=raw_rssi_in,
            freq_mhz=freq_mhz,
            is_moving=bool(self.motion_gate.is_moving),
        )

        # 1. Chặn gai nhiễu Multipath
        clean_rssi, spike_rejected = self.spike.filter(raw_rssi_in, dt, is_fast_lock=is_fast_lock)
        inst_dist = self.model.rssi_to_distance(raw_rssi_in)
        raw_dist = self.model.rssi_to_distance(clean_rssi)

        # 2. Phân loại chuyển động thực
        ref_kf_dist = raw_dist if self.last_kf_dist is None else self.last_kf_dist
        is_moving, delta_d, eff_min_limit, motion_label = self.motion_gate.evaluate(
            z_rssi=clean_rssi,
            raw_dist=raw_dist,
            current_kf_dist=ref_kf_dist,
            dt=dt,
            is_fast_lock=is_fast_lock,
        )

        # 3. Bộ lọc Kalman Tầng 1 & Tầng 2
        kf_rssi, k_gain = self.rssi_kf.update(clean_rssi, is_moving=is_moving, dt=dt, is_fast_lock=is_fast_lock)
        self.last_kf_rssi = kf_rssi

        stage1_dist = self.model.rssi_to_distance(kf_rssi)
        kf_dist, radial_vel = self.dist_kf.update(stage1_dist, is_moving=is_moving, dt=dt, is_fast_lock=is_fast_lock)
        self.last_kf_dist = kf_dist

        if self.smooth_display_dist is None or is_fast_lock:
            self.smooth_display_dist = kf_dist
        else:
            alpha_disp = min(1.0, dt * 22.0)
            self.smooth_display_dist += alpha_disp * (kf_dist - self.smooth_display_dist)

        d_r = self.smooth_display_dist

        # ====================================================================
        # 4. TÍNH TỌA ĐỘ 2D (X, Y) CHUẨN XÁC - KHÔNG SINH GÓC XOAY ẢO
        # ====================================================================
        if self.use_trilateration and secondary_rssi is not None:
            # Giao hội 2 đường tròn (Bilateral Trilateration) giữa PC (0,0) và Anchor phụ (L, 0)
            d_anchor = self.model.rssi_to_distance(secondary_rssi)
            L = max(0.5, self.anchor_baseline_m)
            x_tri = (d_r ** 2 - d_anchor ** 2 + L ** 2) / (2.0 * L)
            x_tri = float(np.clip(x_tri, 0.05 * d_r, 0.98 * d_r))
            y_tri = math.sqrt(max(0.01, d_r ** 2 - x_tri ** 2))
            target_x, target_y = x_tri, y_tri
            target_az = math.degrees(math.atan2(target_x, target_y))
        else:
            # Sử dụng góc thực (từ cảm biến la bàn hoặc góc đặt máy thực tế trên bàn)
            target_az = self.clamp_angle_by_workspace(user_heading_deg)
            rad = math.radians(target_az)
            target_x = d_r * math.sin(rad)
            target_y = d_r * math.cos(rad)

        # Cưỡng bức miền tọa độ theo chế độ không gian để tuyệt đối không xảy ra tọa độ âm sai lệch
        if self.workspace_mode == "POSITIVE_XY":
            target_x = max(0.0, target_x)
            target_y = max(0.0, target_y)
        elif self.workspace_mode == "FRONT_Y":
            target_y = max(0.0, target_y)

        self.azimuth_deg = math.degrees(math.atan2(target_x, max(1e-5, target_y))) if (target_x != 0 or target_y != 0) else target_az

        if self.smooth_x is None or self.smooth_y is None or is_fast_lock:
            self.smooth_x, self.smooth_y = target_x, target_y
        else:
            alpha_xy = min(1.0, dt * 20.0)
            self.smooth_x += alpha_xy * (target_x - self.smooth_x)
            self.smooth_y += alpha_xy * (target_y - self.smooth_y)

        if self.workspace_mode == "POSITIVE_XY":
            self.smooth_x = max(0.0, self.smooth_x)
            self.smooth_y = max(0.0, self.smooth_y)
        elif self.workspace_mode == "FRONT_Y":
            self.smooth_y = max(0.0, self.smooth_y)

        self.hist_raw_rssi.append(raw_rssi_in)
        self.hist_kf_rssi.append(kf_rssi)
        self.hist_raw_dist.append(inst_dist)
        self.hist_kf_dist.append(self.smooth_display_dist)
        if self.step_count % 2 == 0:
            self.hist_xy.append((self.smooth_x, self.smooth_y))

        zone_text, zone_color = self.classify_zone(self.smooth_display_dist)

        return {
            "step": self.step_count,
            "raw_rssi": round(raw_rssi_in, 1),
            "kf_rssi": round(kf_rssi, 2),
            "raw_dist": round(inst_dist, 2),
            "kf_dist": round(self.smooth_display_dist, 2),
            "radial_vel": round(radial_vel, 2),
            "azimuth_deg": round(self.azimuth_deg, 1),
            "x": round(self.smooth_x, 3),
            "y": round(self.smooth_y, 3),
            "is_moving": is_moving,
            "spike_rejected": spike_rejected,
            "delta_d": round(delta_d, 2),
            "eff_min_limit": round(eff_min_limit, 2),
            "motion_label": motion_label,
            "zone_text": zone_text,
            "zone_color": zone_color,
            "k_gain": round(k_gain, 3),
            "auto_a": round(self.model.a, 1),
            "auto_n": round(self.model.n, 2),
            "freq_mhz": freq_mhz,
            "has_live_compass": has_live_compass,
        }


# ============================================================================
# PHẦN 5: GIAO DIỆN ĐỒ HỌA 60 FPS VỚI HỆ TRỤC TỌA ĐỘ DƯƠNG CHUẨN XÁC
# ============================================================================
def run_60fps_dashboard(collector: HighSpeedSignalCollector):
    import tkinter as tk
    from tkinter import ttk

    FRAME_PERIOD_MS = 16
    AUTO_LABEL = "Chưa gán hotspot"

    root = tk.Tk()
    root.title("Radar 2D Định Vị Điện Thoại 60 FPS | Chuẩn Hóa Góc Thực & Khóa Tọa Độ Dương (X ≥ 0, Y ≥ 0)")
    root.geometry("1280x810")
    root.configure(bg="#0d131f")

    locators = [
        Realtime2DHotspotLocator(dev_index=0),
        Realtime2DHotspotLocator(dev_index=1),
        Realtime2DHotspotLocator(dev_index=2),
    ]
    colors = ["#00ff88", "#00d4ff", "#ffcc66"]
    device_names = ["Điện thoại 1", "Điện thoại 2", "Điện thoại 3"]
    assigned: List[Optional[str]] = [None, None, None]
    last_results: List[Optional[Dict[str, object]]] = [None, None, None]

    # --- Thanh điều khiển 1: Gán Hotspot ---
    top_bar = tk.Frame(root, bg="#161f33", pady=6, padx=12)
    top_bar.pack(fill=tk.X)
    tk.Label(top_bar, text="GÁN HOTSPOT:", font=("Segoe UI", 9, "bold"),
             fg="#94a3b8", bg="#161f33").pack(side=tk.LEFT, padx=(0, 6))

    device_vars, net_combos = [], []
    for i in range(3):
        tk.Label(top_bar, text=f"ĐT{i+1}:", fg=colors[i], bg="#161f33",
                 font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(4, 2))
        var = tk.StringVar(value=AUTO_LABEL)
        combo = ttk.Combobox(top_bar, textvariable=var, width=18, state="readonly")
        combo.pack(side=tk.LEFT, padx=3)
        device_vars.append(var)
        net_combos.append(combo)

    lbl_fps = tk.Label(top_bar, text="FPS: 60.0", font=("Consolas", 10, "bold"),
                       fg="#00ff88", bg="#161f33", padx=8)
    lbl_fps.pack(side=tk.RIGHT)
    lbl_model = tk.Label(
        top_bar,
        text="Auto-Calib: A(1m)=-43.5 dBm | n=2.25",
        font=("Consolas", 10, "bold"), fg="#ffcc00", bg="#161f33", padx=8
    )
    lbl_model.pack(side=tk.RIGHT)

    # --- Thanh điều khiển 2: Chọn Thiết bị, Góc Thực 2D & Chế Độ Miền Tọa Độ ---
    ctrl_bar = tk.Frame(root, bg="#111a2e", pady=5, padx=12)
    ctrl_bar.pack(fill=tk.X)
    tk.Label(ctrl_bar, text="THIẾT BỊ:", fg="#38bdf8", bg="#111a2e",
             font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
    selected_var = tk.StringVar(value=device_names[0])
    selected_combo = ttk.Combobox(ctrl_bar, textvariable=selected_var, values=device_names,
                                  width=13, state="readonly")
    selected_combo.pack(side=tk.LEFT, padx=6)

    def selected_index() -> int:
        try:
            return device_names.index(selected_var.get())
        except ValueError:
            return 0

    tk.Label(ctrl_bar, text="GÓC ĐẶT MÁY (°):", fg="#00ffcc", bg="#111a2e",
             font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(6, 4))
    azimuth_var = tk.DoubleVar(value=DEFAULT_POSITIVE_ANGLES[0])

    def on_azimuth_slide(_val=None):
        idx = selected_index()
        ang = locators[idx].clamp_angle_by_workspace(float(azimuth_var.get()))
        azimuth_var.set(round(ang, 1))
        collector.set_manual_azimuth(idx, ang)
        locators[idx].azimuth_deg = ang

    az_scale = tk.Scale(ctrl_bar, from_=0, to=90, resolution=1, orient=tk.HORIZONTAL,
                        variable=azimuth_var, command=on_azimuth_slide, bg="#111a2e", fg="#00ffcc",
                        highlightthickness=0, troughcolor="#1e293b", length=140)
    az_scale.pack(side=tk.LEFT, padx=4)

    def on_device_combo_change(_event=None):
        idx = selected_index()
        azimuth_var.set(round(locators[idx].azimuth_deg, 1))

    selected_combo.bind("<<ComboboxSelected>>", on_device_combo_change)

    # Bộ chọn Chế độ Miền Tọa Độ (Mặc định: Tọa độ Dương X >= 0, Y >= 0)
    tk.Label(ctrl_bar, text=" | MIỀN TỌA ĐỘ:", fg="#ffcc00", bg="#111a2e",
             font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(6, 4))
    mode_labels = [
        "Tọa độ Dương (X ≥ 0, Y ≥ 0)",
        "Phía Trước PC (Y ≥ 0)",
        "Toàn Mặt Phẳng 360°",
    ]
    mode_var = tk.StringVar(value=mode_labels[0])
    mode_combo = ttk.Combobox(ctrl_bar, textvariable=mode_var, values=mode_labels, width=23, state="readonly")
    mode_combo.pack(side=tk.LEFT, padx=4)

    def on_workspace_mode_change(_event=None):
        sel = mode_var.get()
        if sel == mode_labels[0]:
            w_mode = "POSITIVE_XY"
            az_scale.config(from_=0, to=90)
        elif sel == mode_labels[1]:
            w_mode = "FRONT_Y"
            az_scale.config(from_=-90, to=90)
        else:
            w_mode = "FULL_360"
            az_scale.config(from_=-180, to=180)

        for idx_dev, loc in enumerate(locators):
            loc.workspace_mode = w_mode
            clamped = loc.clamp_angle_by_workspace(loc.azimuth_deg)
            loc.azimuth_deg = clamped
            collector.set_manual_azimuth(idx_dev, clamped)
        azimuth_var.set(round(locators[selected_index()].azimuth_deg, 1))

    mode_combo.bind("<<ComboboxSelected>>", on_workspace_mode_change)

    tri_var = tk.BooleanVar(value=False)
    def on_tri_toggle():
        for loc in locators:
            loc.use_trilateration = bool(tri_var.get())

    tk.Checkbutton(ctrl_bar, text="Giao hội Đa BSSID", variable=tri_var,
                   command=on_tri_toggle, bg="#111a2e", fg="#e2e8f0", selectcolor="#1e293b",
                   activebackground="#111a2e", font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=6)

    def on_instant_relock():
        idx = selected_index()
        locators[idx].soft_reset()

    tk.Button(ctrl_bar, text="Khóa Lại Tức Thì", command=on_instant_relock,
              bg="#0284c7", fg="white", font=("Segoe UI", 8, "bold"), relief=tk.FLAT,
              padx=9, pady=2).pack(side=tk.RIGHT, padx=4)

    # --- Thanh điều khiển 3: Ngưỡng chuyển động ---
    gate_bar = tk.Frame(root, bg="#111a2e", pady=4, padx=12)
    gate_bar.pack(fill=tk.X)
    tk.Label(gate_bar, text="ĐỘ NHẠY DI CHUYỂN:", font=("Segoe UI", 9, "bold"),
             fg="#38bdf8", bg="#111a2e").pack(side=tk.LEFT, padx=(0, 8))
    min_dist_var = tk.DoubleVar(value=0.35)
    min_dbm_var = tk.DoubleVar(value=2.2)
    lbl_gate_info = tk.Label(
        gate_bar,
        text="Click/Kéo trực tiếp trên Radar để đặt đúng góc thực tế của từng điện thoại | Đã khóa X ≥ 0, Y ≥ 0",
        font=("Consolas", 9, "bold"), fg="#00ff88", bg="#111a2e"
    )

    def on_threshold_change(_val=None):
        md, mb = float(min_dist_var.get()), float(min_dbm_var.get())
        for loc in locators:
            loc.motion_gate.set_min_thresholds(md, mb)
        lbl_gate_info.config(
            text=f"Ngưỡng: >= {md:.2f}m & >= {mb:.1f}dBm | Click/Kéo trên Radar để đặt đúng hướng thực tế của điện thoại"
        )

    tk.Label(gate_bar, text="Min D (m):", fg="#cbd5e1", bg="#111a2e").pack(side=tk.LEFT)
    tk.Scale(gate_bar, from_=0.15, to=1.00, resolution=0.05, orient=tk.HORIZONTAL,
             variable=min_dist_var, command=on_threshold_change, bg="#111a2e", fg="#00ffcc",
             highlightthickness=0, troughcolor="#1e293b", length=110).pack(side=tk.LEFT, padx=4)
    tk.Label(gate_bar, text="Min RSSI (dBm):", fg="#cbd5e1", bg="#111a2e").pack(side=tk.LEFT)
    tk.Scale(gate_bar, from_=1.2, to=4.5, resolution=0.2, orient=tk.HORIZONTAL,
             variable=min_dbm_var, command=on_threshold_change, bg="#111a2e", fg="#ffcc00",
             highlightthickness=0, troughcolor="#1e293b", length=110).pack(side=tk.LEFT, padx=4)
    lbl_gate_info.pack(side=tk.LEFT, padx=6)

    # --- Thanh trạng thái ---
    status_bar = tk.Frame(root, bg="#0b101b", pady=6, padx=12)
    status_bar.pack(fill=tk.X)
    lbl_status = tk.Label(status_bar, text="Đang khóa tín hiệu tức thì...", font=("Consolas", 10, "bold"),
                          fg="#00ffcc", bg="#0b101b", anchor="w")
    lbl_status.pack(side=tk.LEFT, fill=tk.X, expand=True)
    lbl_mode = tk.Label(status_bar, text="Chế độ: Tọa độ Dương (X ≥ 0, Y ≥ 0)", font=("Consolas", 10, "bold"),
                        fg="#38bdf8", bg="#0b101b")
    lbl_mode.pack(side=tk.RIGHT)

    # --- Khung Canvas chính ---
    body = tk.Frame(root, bg="#0d131f")
    body.pack(fill=tk.BOTH, expand=True, padx=10, pady=6)
    radar_cv = tk.Canvas(body, width=540, height=540, bg="#080c14",
                         highlightthickness=1, highlightbackground="#1e293b", cursor="crosshair")
    radar_cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 8))
    chart_cv = tk.Canvas(body, width=680, height=540, bg="#080c14",
                         highlightthickness=1, highlightbackground="#1e293b")
    chart_cv.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

    # Cho phép Click / Kéo trực tiếp điểm điện thoại trên Radar để đặt đúng hướng thực tế
    def on_radar_click_or_drag(event):
        cw = max(540, radar_cv.winfo_width())
        ch = max(540, radar_cv.winfo_height())
        cx, cy = cw // 2, ch // 2
        dx = event.x - cx
        dy = cy - event.y
        if abs(dx) < 3 and abs(dy) < 3:
            return
        raw_angle_deg = math.degrees(math.atan2(dx, dy))
        idx = selected_index()
        clamped_angle = locators[idx].clamp_angle_by_workspace(raw_angle_deg)
        azimuth_var.set(round(clamped_angle, 1))
        collector.set_manual_azimuth(idx, clamped_angle)
        locators[idx].azimuth_deg = clamped_angle

    radar_cv.bind("<Button-1>", on_radar_click_or_drag)
    radar_cv.bind("<B1-Motion>", on_radar_click_or_drag)

    last_net_keys: Tuple[str, ...] = ()

    def refresh_network_list(nets: Dict[str, Dict[str, object]]):
        nonlocal last_net_keys
        sorted_keys = tuple(sorted(nets.keys(), key=str.lower))
        if sorted_keys != last_net_keys:
            last_net_keys = sorted_keys
            items = [AUTO_LABEL] + list(sorted_keys)
            for combo in net_combos:
                combo["values"] = items

        if all(a is None for a in assigned) and nets:
            best_ssid = max(nets.values(), key=lambda x: float(x["rssi"]))["ssid"]
            assigned[0] = str(best_ssid)
            device_vars[0].set(str(best_ssid))
            locators[0].soft_reset()

    def make_select_handler(i: int):
        def on_select(_event=None):
            val = device_vars[i].get()
            if val == AUTO_LABEL:
                assigned[i] = None
                locators[i].soft_reset()
                return
            for j in range(3):
                if j != i and assigned[j] == val:
                    assigned[j] = None
                    device_vars[j].set(AUTO_LABEL)
                    locators[j].soft_reset()
            assigned[i] = val
            locators[i].soft_reset()
        return on_select

    for i, combo in enumerate(net_combos):
        combo.bind("<<ComboboxSelected>>", make_select_handler(i))

    def draw_series(cv, data, x0, y0, w, h, vmin, vmax, color, width=2):
        if len(data) < 2:
            return
        pts = []
        span = max(1e-3, vmax - vmin)
        denom = max(1, len(data) - 1)
        for j, val in enumerate(data):
            px = x0 + (j / denom) * w
            py = y0 + h - ((max(vmin, min(vmax, val)) - vmin) / span) * h
            pts.extend([px, py])
        cv.create_line(*pts, fill=color, width=width, smooth=True)

    fps_ema = 60.0
    last_frame_ts = time.perf_counter()
    frame_counter = 0

    def tick_60fps():
        nonlocal fps_ema, last_frame_ts, frame_counter
        now_ts = time.perf_counter()
        dt_frame = max(1e-4, now_ts - last_frame_ts)
        last_frame_ts = now_ts
        fps_ema = 0.92 * fps_ema + 0.08 * (1.0 / dt_frame)
        frame_counter += 1

        snap = collector.get_realtime_snapshot()
        nets = snap["networks"]
        headings = snap["headings"]
        live_compass = snap["live_compass"]

        if frame_counter == 1 or frame_counter % 15 == 0:
            refresh_network_list(nets)
            lbl_fps.config(text=f"FPS: {min(60.0, fps_ema):.1f}")

        active_texts = []
        for i in range(3):
            ssid = assigned[i]
            if ssid and ssid in nets:
                net_info = nets[ssid]
                res = locators[i].step_realtime(
                    raw_rssi_in=float(net_info["rssi"]),
                    freq_mhz=int(net_info.get("freq_mhz", 2412)),
                    user_heading_deg=float(headings.get(i, DEFAULT_POSITIVE_ANGLES[i])),
                    has_live_compass=bool(live_compass.get(i, False)),
                    secondary_rssi=net_info.get("secondary_rssi"),
                )
                last_results[i] = res
                active_texts.append(
                    f"ĐT{i+1}[{ssid[:10]}]: {res['kf_dist']:.2f}m | ∠{res['azimuth_deg']:.0f}° | (X={res['x']:+.2f}m, Y={res['y']:+.2f}m)"
                )
            else:
                last_results[i] = None
                active_texts.append(f"ĐT{i+1}: {'Mất tín hiệu' if ssid else 'Chưa gán'}")

        lbl_status.config(text="   ||   ".join(active_texts))

        sel_idx = selected_index()
        sel_res = last_results[sel_idx]
        if sel_res:
            band_str = "5GHz" if sel_res["freq_mhz"] >= 4900 else "2.4GHz"
            lbl_model.config(
                text=f"Auto-Calib ({band_str}): A(1m)={sel_res['auto_a']:.1f}dBm | n={sel_res['auto_n']:.2f}"
            )

        # ====================================================================
        # VẼ RADAR 2 CHIỀU (60 FPS) - TÔ SÁNG MIỀN TỌA ĐỘ DƯƠNG (X >= 0, Y >= 0)
        # ====================================================================
        radar_cv.delete("all")
        cw, ch = max(540, radar_cv.winfo_width()), max(540, radar_cv.winfo_height())
        cx, cy = cw // 2, ch // 2
        max_r = max(5, math.ceil(max([r["kf_dist"] for r in last_results if r] + [4.0])))
        scale = (min(cw, ch) * 0.41) / max_r

        # Tô nổi bật vùng Góc Phần Tư Dương (X >= 0, Y >= 0) ở góc trên bên phải
        if locators[0].workspace_mode == "POSITIVE_XY":
            radar_cv.create_rectangle(cx, 35, cw - 18, cy, fill="#0c192c", outline="#1e3a5f", width=1)
            radar_cv.create_text(
                cx + (cw - cx) // 2, 48,
                text="VÙNG TỌA ĐỘ DƯƠNG (X ≥ 0, Y ≥ 0)",
                fill="#38bdf8", font=("Consolas", 8, "bold")
            )
        elif locators[0].workspace_mode == "FRONT_Y":
            radar_cv.create_rectangle(18, 35, cw - 18, cy, fill="#0c192c", outline="#1e3a5f", width=1)

        radar_cv.create_text(cx, 18, text="RADAR ĐỊNH VỊ 2 CHIỀU (60 FPS) | HIỂN THỊ TỌA ĐỘ THỰC (X, Y)",
                             fill="#e2e8f0", font=("Segoe UI", 10, "bold"))

        for angle_grid in range(0, 360, 30):
            rad_g = math.radians(angle_grid)
            gx = cx + (max_r * scale) * math.sin(rad_g)
            gy = cy - (max_r * scale) * math.cos(rad_g)
            radar_cv.create_line(cx, cy, gx, gy, fill="#162032", dash=(2, 4))

        # Vẽ trục tọa độ X và Y rõ ràng
        radar_cv.create_line(18, cy, cw - 18, cy, fill="#334155", width=2)
        radar_cv.create_line(cx, 35, cx, ch - 18, fill="#334155", width=2)
        radar_cv.create_text(cw - 35, cy - 10, text="+X (m)", fill="#00ffcc", font=("Consolas", 9, "bold"))
        radar_cv.create_text(cx + 28, 45, text="+Y (m)", fill="#00ffcc", font=("Consolas", 9, "bold"))

        for r_m in range(1, int(max_r) + 1):
            rp = r_m * scale
            radar_cv.create_oval(cx - rp, cy - rp, cx + rp, cy + rp, outline="#1e293b", dash=(3, 3))
            radar_cv.create_text(cx + 16, cy - rp + 10, text=f"{r_m}m", fill="#64748b", font=("Consolas", 8))

        radar_cv.create_polygon(cx, cy - 10, cx - 9, cy + 8, cx + 9, cy + 8, fill="#ff3366", outline="white")
        radar_cv.create_text(cx, cy + 20, text="PC (0,0)", fill="#ff3366", font=("Consolas", 9, "bold"))

        for i, res in enumerate(last_results):
            if res is None:
                continue

            trail = list(locators[i].hist_xy)
            if len(trail) >= 2:
                trail_pts = []
                for tx, ty in trail:
                    trail_pts.extend([cx + tx * scale, cy - ty * scale])
                radar_cv.create_line(*trail_pts, fill="#334155", width=2, smooth=True)

            px = cx + res["x"] * scale
            py = cy - res["y"] * scale

            # Vẽ hình chiếu vuông góc xuống trục +X và +Y để kiểm chứng trực quan tọa độ dương
            if res["x"] >= 0 and res["y"] >= 0:
                radar_cv.create_line(px, py, px, cy, fill="#1e293b", dash=(2, 2))
                radar_cv.create_line(px, py, cx, py, fill="#1e293b", dash=(2, 2))

            radar_cv.create_line(cx, cy, px, py, fill=colors[i], dash=(2, 2), width=1)

            if res["spike_rejected"]:
                radar_cv.create_oval(px - 15, py - 15, px + 15, py + 15, outline="#ff5555", width=2)

            radar_cv.create_oval(px - 9, py - 9, px + 9, py + 9, fill=colors[i], outline="white", width=2)
            radar_cv.create_text(
                px, py - 20,
                text=f"ĐT{i+1}: {res['kf_dist']:.2f}m | ∠{res['azimuth_deg']:.0f}°\n(X={res['x']:+.2f}, Y={res['y']:+.2f})",
                fill=colors[i], font=("Consolas", 8, "bold"), justify=tk.CENTER
            )

        # ====================================================================
        # VẼ ĐỒ THỊ THỜI GIAN THỰC 60 FPS
        # ====================================================================
        chart_cv.delete("all")
        x0, gw, gh = 48, max(300, chart_cv.winfo_width() - 70), 195
        y1 = 35
        chart_cv.create_rectangle(x0, y1, x0 + gw, y1 + gh, outline="#1e293b")
        chart_cv.create_text(
            x0 + 8, y1 + 14, anchor="w",
            text=f"1. Tín hiệu RSSI 60FPS - Thiết bị {sel_idx+1} (Vàng: Thô | Xanh ngọc: Kalman Hội Tụ Nhanh)",
            fill="#e2e8f0", font=("Consolas", 9, "bold")
        )
        if sel_res:
            draw_series(chart_cv, list(locators[sel_idx].hist_raw_rssi), x0, y1, gw, gh, -95, -20, "#ffaa00", 1)
            draw_series(chart_cv, list(locators[sel_idx].hist_kf_rssi), x0, y1, gw, gh, -95, -20, "#00ffcc", 2)

        y2 = y1 + gh + 40
        chart_cv.create_rectangle(x0, y2, x0 + gw, y2 + gh, outline="#1e293b")
        chart_cv.create_text(
            x0 + 8, y2 + 14, anchor="w",
            text=f"2. Khoảng cách 60FPS - Thiết bị {sel_idx+1} (Đỏ: Thô | Xanh lá: Tự Động Hiệu Chuẩn & Chống Vọt)",
            fill="#e2e8f0", font=("Consolas", 9, "bold")
        )
        if sel_res:
            draw_series(chart_cv, list(locators[sel_idx].hist_raw_dist), x0, y2, gw, gh, 0, max_r, "#ff5555", 1)
            draw_series(chart_cv, list(locators[sel_idx].hist_kf_dist), x0, y2, gw, gh, 0, max_r, "#00ff88", 3)
            lbl_mode.config(
                text=f"{sel_res['motion_label']} | X={sel_res['x']:+.2f}m, Y={sel_res['y']:+.2f}m",
                fg="#ff5555" if sel_res["spike_rejected"] else ("#ffcc00" if sel_res["is_moving"] else "#00ff88")
            )
        else:
            lbl_mode.config(text="Chọn hotspot cho thiết bị trên thanh công cụ", fg="#38bdf8")

        root.after(FRAME_PERIOD_MS, tick_60fps)

    def on_closing():
        collector.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.after(FRAME_PERIOD_MS, tick_60fps)
    root.mainloop()


if __name__ == "__main__":
    signal_collector = HighSpeedSignalCollector(udp_port=5005, http_port=8080)
    run_60fps_dashboard(signal_collector)