# Hotspot Kalman Locator 2D (`DPS.py`)

[![Python](https://img.shields.io/badge/Python-3.8%2B-black?style=flat-square&logo=python)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Windows%2010%2F11%20(wlanapi.dll)-black?style=flat-square&logo=windows)](https://learn.microsoft.com/en-us/windows/win32/nativewifi/portal)
[![DSP](https://img.shields.io/badge/DSP-Two--Stage%20Kalman%20%2B%20Hampel-black?style=flat-square)]()
[![UI](https://img.shields.io/badge/Dashboard-60%20FPS%20Real--Time%20Radar-black?style=flat-square)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-black?style=flat-square)](./LICENSE)

> **Real-Time 2D Wi-Fi Hotspot Indoor Positioning & Digital Signal Processing (DSP) System** powered by Direct Windows Native WLAN API (`wlanapi.dll`), a Two-Stage Cascaded Kalman Filtering Pipeline, and a 60 FPS Interactive Radar + Mobile Web Compass Beacon.

---

## 1. Overview

**Hotspot Kalman Locator 2D (`DPS.py`)** is a self-contained, zero-external-dependency Python system that estimates the real-time radial distance and 2D spatial coordinates $(X, Y)$ of up to **three mobile Wi-Fi hotspots simultaneously** (`ĐT1`, `ĐT2`, `ĐT3`) relative to a Windows PC station $(0, 0)$.

Unlike conventional Wi-Fi RSSI scanners that rely on slow command-line `netsh wlan show networks` polling (which suffers from OS-level caching delays of $3\text{--}5\text{ s}$) and naïve single-stage log-distance formulas, this project implements a complete **hardware-to-GUI signal processing architecture**:

1. **Direct Native Win32 Hardware Scanning (`wlanapi.dll`)**: Bypasses CLI subprocess bottlenecks via `ctypes` (`WlanScan` + `WlanGetNetworkBssList`) to read raw per-BSSID link quality and RSSI (dBm) directly from the wireless NIC driver every $0.35\text{ s}$.
2. **Embedded Mobile Web Compass Server (`HTTP Port 8080`)**: Hosts a mobile web application (`WEB_BEACON_HTML`) that captures smartphone `DeviceOrientationEvent` magnetometer/gyroscope heading at $12.5\text{ Hz}$ ($80\text{ ms}$) with one-tap **`"Tare 0°"`** calibration toward the PC screen.
3. **Six-Layer Real-Time DSP Pipeline**:
   - **Layer 1**: **Hampel Non-Linear Outlier Filter** ($3\sigma_{\text{MAD}}$ sliding window) to reject sudden non-Gaussian fading spikes.
   - **Layer 2**: **Stage-1 Asymmetric 1D RSSI Kalman Filter** with dual innovation gating ($2.8\,\text{dBm}$) and $4.2\times$ multipath fade damping.
   - **Layer 3**: **Dual-Slope Breakpoint Log-Distance Path-Loss Inversion** ($d_{\text{bp}} = 2.0\text{ m}$, $n_{\text{near}} = 2.0$, $n_{\text{far}} = 2.9$).
   - **Layer 4**: **Stage-2 Constant-Velocity (CV) 2-State Kinematic Kalman Filter** ($\mathbf{x}_k = [d_k, v_k]^T$) with discrete white-noise acceleration process covariance $\mathbf{Q}(\Delta t)$ and hard velocity clamping ($\pm 2.5\text{ m/s}$).
   - **Layer 5**: **Hand-Held Motion Discriminator (`HandHeldMotionDiscriminator`)** featuring dual-threshold hysteresis and a **Fast-Lock (`< 0.45 s`)** initial acquisition state machine.
   - **Layer 6**: **2D Polar Azimuth Projection & Multi-BSSID Bilateral Trilateration** across three selectable spatial domains (`POSITIVE_XY`, `FRONT_Y`, `FULL_360`).

---

## 2. System Interface & Architecture Previews

### 2.1. 60 FPS Desktop Radar & Dual Oscilloscope Dashboard (`run_60fps_dashboard`)
![60 FPS Desktop Dashboard](./docs/images/fig_5_1_desktop_dashboard.png)

### 2.2. Real-Time DSP Signal Waveforms (`draw_chart_60fps`)
![Dual Oscilloscope Waveforms](./docs/images/fig_5_2_oscilloscope_detail.png)

### 2.3. Mobile Web Compass Beacon (`WEB_BEACON_HTML` on TCP Port 8080)
![Mobile Web Beacon States](./docs/images/fig_6_1_web_beacon_states.png)

### 2.4. Web Control Anatomy & `"Tare 0°"` Relative Azimuth Geometry
![Web Controls and Tare Geometry](./docs/images/fig_6_2_web_controls_and_tare_geometry.png)

### 2.5. Desktop Toolbar Callout Map & Radar Mouse Calibration Modes
![Desktop Controls Anatomy](./docs/images/fig_6_3_desktop_controls_anatomy.png)
![Radar Mouse and Quadrant Modes](./docs/images/fig_6_4_radar_mouse_and_quadrant_modes.png)

---

## 3. Mathematical Formulation

### 3.1. Dual-Slope Log-Distance Path-Loss Model
To account for the indoor transition between the near-field/Fresnel line-of-sight zone and the multipath-dominated far field at breakpoint distance $d_{\text{bp}} = 2.0\text{ m}$:

$$
d(\text{RSSI}) =
\begin{cases}
10^{\frac{\text{RSSI}_{1\text{m}} - \text{RSSI}}{10 \cdot n_{\text{near}}}}, & \text{if } \text{RSSI} \ge \text{RSSI}_{\text{bp}} \\[8pt]
d_{\text{bp}} \cdot 10^{\frac{\text{RSSI}_{\text{bp}} - \text{RSSI}}{10 \cdot n_{\text{far}}}}, & \text{if } \text{RSSI} < \text{RSSI}_{\text{bp}}
\end{cases}
$$

where $\text{RSSI}_{\text{bp}} = \text{RSSI}_{1\text{m}} - 10 \cdot n_{\text{near}} \log_{10}(d_{\text{bp}})$.

### 3.2. Stage-2 Constant-Velocity (CV) Kinematic Kalman Filter
State vector $\mathbf{x}_k = [d_k, v_k]^T$, transition matrix $\mathbf{F}(\Delta t)$, and continuous-to-discrete acceleration process noise $\mathbf{Q}(\Delta t)$:

$$
\mathbf{F}(\Delta t) =
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix},
\qquad
\mathbf{Q}(\Delta t) = q_v
\begin{bmatrix}
\frac{\Delta t^3}{3} & \frac{\Delta t^2}{2} \\
\frac{\Delta t^2}{2} & \Delta t
\end{bmatrix}
$$

---

## 4. Repository Structure

```text
DSP/
├── DPS.py                          # Main application (1,423 lines: Native WLAN API + DSP + Web Server + 60 FPS GUI)
├── TECHNICAL_REPORT_DPS_EN.docx    # Comprehensive 65-page IEEE/University Technical Report (Full English, B&W)
├── BÁO CÁO KỸ THUẬT HỆ THỐNG.docx  # Original System Technical Report Cover & Template
├── docs/
│   └── images/                     # High-resolution architectural & UI schematic diagrams
│       ├── fig_5_1_desktop_dashboard.png
│       ├── fig_5_2_oscilloscope_detail.png
│       ├── fig_6_1_web_beacon_states.png
│       ├── fig_6_2_web_controls_and_tare_geometry.png
│       ├── fig_6_3_desktop_controls_anatomy.png
│       └── fig_6_4_radar_mouse_and_quadrant_modes.png
├── requirements.txt                # Runtime & optional scientific calibration dependencies
├── LICENSE                         # MIT License
└── README.md                       # Project documentation
```

---

## 5. Quick Start & Usage

### Prerequisites
- **Operating System**: Windows 10 or Windows 11 (x64) with an active Wi-Fi adapter (`wlanapi.dll`).
- **Python**: Python `3.8+` (uses only Python Standard Library modules: `ctypes`, `tkinter`, `threading`, `http.server`, `math`, `collections`).
- **Privileges**: Run PowerShell / Command Prompt as **Administrator** recommended so `WlanScan` can trigger immediate hardware driver scans without OS throttling.

### Running the Application
```powershell
# Clone the repository
git clone https://github.com/Benniededepresso/DSP.git
cd DSP

# Launch the 60 FPS Radar Dashboard & Embedded Web Beacon Server
python DPS.py
```

### Connecting Mobile Hotspots (`ĐT1`, `ĐT2`, `ĐT3`)
1. Enable **Personal Hotspot (Wi-Fi Tethering)** on 1 to 3 smartphones.
2. In the top toolbar (`GÁN HOTSPOT`), select each phone's SSID from the dropdown (`ĐT 1`, `ĐT 2`, `ĐT 3`). The strongest nearby hotspot is automatically assigned to `ĐT 1` on startup.
3. **Optional 2D True-Azimuth Sync**:
   - Connect your phone or secondary browser to the same LAN and open `http://<PC-LAN-IP>:8080` (displayed in the top-right badge of the dashboard).
   - Tap **`1. Bật Cảm Biến Góc`** to enable `DeviceOrientationEvent`.
   - Point the top of the phone toward the PC screen and tap **`2. Đặt Hướng Hiện Tại = 0° (Chính diện PC)`** to stream live 2D azimuth angles at $12.5\text{ Hz}$.
   - Alternatively, **Left-Click and Drag** directly on the 2D Radar canvas or adjust the **`GÓC ĐẶT MÁY (°)`** slider.

---

## 6. Technical Report (`TECHNICAL_REPORT_DPS_EN.docx`)

Included in this repository is [**`TECHNICAL_REPORT_DPS_EN.docx`**](./TECHNICAL_REPORT_DPS_EN.docx), a **65-page academic technical report** covering:
- **Chapter I**: Introduction, System Context, and Architectural Overview
- **Chapter II**: Theoretical Foundations & Mathematical Modeling
- **Chapter III**: Real-Time DSP & Two-Stage Kalman Filtering Architecture
- **Chapter IV**: Low-Level Windows Native WLAN Hardware Integration (`wlanapi.dll`) & HTTP Telemetry Server
- **Chapter V**: Multi-Threaded Concurrency & 60 FPS Radar Visualization Engine
- **Chapter VI**: Web Interface & Complete User Control Reference
- **Chapter VII**: Experimental Evaluation, 1D Radial Performance vs. 2D Inter-Phone Unobservability Analysis & Multi-Modal Remediation Strategies
- **Chapter VIII**: Conclusions & Future Research Roadmap

---

## 7. License

This project is licensed under the **MIT License** — see the [LICENSE](./LICENSE) file for details.
