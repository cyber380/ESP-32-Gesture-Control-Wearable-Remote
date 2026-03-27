# ============================================================
#  controller_main.py  –  ESP32-C3  (Gesture Controller)
#  v4.0 – Redesigned OLED UI
#
#  UI screens:
#    BOOT       → animated progress bar + car logo
#    WAIT PIVOT → pulsing dot + instruction panel
#    PIVOT SET  → checkmark + locked angles
#    DRIVING    → live crosshair + big icon + tilt bars + HUD
#    NO SIGNAL  → bold X with MAC hint
#    IMU ERROR  → warning icon + retry counter
# ============================================================

import time
import struct
import math
import network
import espnow
from machine import SoftI2C, Pin, WDT

try:
    from config import i2c_mpu, i2c_oled, BTN_A, BTN_B, BTN_C
    _CONFIG_LOADED = True
except ImportError:
    _CONFIG_LOADED = False

# ╔══════════════════════════════════════════════════════════╗
#  PIN CONFIGURATION
# ╚══════════════════════════════════════════════════════════╝
OLED_SDA      = 6
OLED_SCL      = 7
OLED_ADDR     = 0x3C
OLED_CONTRAST = 200
OLED_FREQ     = 400_000

MPU_SDA       = 6
MPU_SCL       = 7
MPU_FREQ      = 400_000
BTN_B_PIN     = 4

# ╔══════════════════════════════════════════════════════════╗
#  TUNING CONSTANTS
# ╚══════════════════════════════════════════════════════════╝
TILT_THRESH      = 12.0
CIRCLE_GYRO_MAG  = 150.0
CONFIRM_FRAMES   = 3
ALPHA            = 0.92
HEARTBEAT_MS     = 300
PIVOT_SAMPLES    = 30
LOOP_MS          = 20
GYRO_BUF_SIZE    = 5
WDT_TIMEOUT_MS   = 8000
MAX_SEND_FAILS   = 3
IMU_RETRY_LIMIT  = 5
OLED_REDRAW_MS   = 50

RC_CAR_MAC = b'\x8c\x4f\x00\xac\xbb\xe4'

# ══════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════
def log(msg):
    print("[{:8d}] {}".format(time.ticks_ms(), msg))

# ══════════════════════════════════════════════════════════════
#  I2C + OLED INIT
# ══════════════════════════════════════════════════════════════
if not _CONFIG_LOADED:
    i2c_oled = SoftI2C(scl=Pin(OLED_SCL), sda=Pin(OLED_SDA), freq=OLED_FREQ)
    i2c_mpu  = i2c_oled if (MPU_SDA==OLED_SDA and MPU_SCL==OLED_SCL) else \
               SoftI2C(scl=Pin(MPU_SCL), sda=Pin(MPU_SDA), freq=MPU_FREQ)
    BTN_B = Pin(BTN_B_PIN, Pin.IN, Pin.PULL_UP)
    BTN_A = BTN_B; BTN_C = BTN_B

import ssd1306 as _ssd_mod

_oled   = None
OLED_OK = False

def _oled_init():
    global _oled, OLED_OK
    def _try(addr):
        global _oled, OLED_OK
        try:
            obj = _ssd_mod.SSD1306_I2C(128, 64, i2c_oled, addr=addr)
            obj.fill(0); obj.show()
            _oled = obj; OLED_OK = True
            try: _oled.contrast(OLED_CONTRAST)
            except Exception: pass
            log("OLED OK 0x{:02X}".format(addr)); return True
        except Exception as e:
            log("OLED 0x{:02X} fail: {}".format(addr, e)); return False
    if _try(OLED_ADDR): return True
    alt = 0x3D if OLED_ADDR == 0x3C else 0x3C
    if _try(alt): return True
    try:
        for addr in i2c_oled.scan():
            if addr in (0x3C, 0x3D):
                if _try(addr): return True
    except Exception: pass
    log("OLED disabled"); return False

_oled_init()

def _show():
    if not OLED_OK: return
    try: _oled.show()
    except Exception as e: log("show err: {}".format(e))

# ══════════════════════════════════════════════════════════════
#  LOW-LEVEL DRAWING PRIMITIVES
# ══════════════════════════════════════════════════════════════
def _px(x, y, c=1):
    if 0 <= x < 128 and 0 <= y < 64: _oled.pixel(x, y, c)

def _hline(x, y, w, c=1):
    if OLED_OK and w > 0: _oled.fill_rect(x, y, w, 1, c)

def _vline(x, y, h, c=1):
    if OLED_OK and h > 0: _oled.fill_rect(x, y, 1, h, c)

def _rect(x, y, w, h, c=1):
    if OLED_OK: _oled.rect(x, y, w, h, c)

def _frect(x, y, w, h, c=1):
    if OLED_OK and w > 0 and h > 0: _oled.fill_rect(x, y, w, h, c)

def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def _text(s, x, y, c=1):
    if OLED_OK: _oled.text(s, x, y, c)

# ── Bresenham circle ─────────────────────────────────────────
def _circle(cx, cy, r, c=1):
    x, y, d = r, 0, 1 - r
    while x >= y:
        for pts in ((cx+x,cy+y),(cx-x,cy+y),(cx+x,cy-y),(cx-x,cy-y),
                    (cx+y,cy+x),(cx-y,cy+x),(cx+y,cy-x),(cx-y,cy-x)):
            _px(pts[0], pts[1], c)
        y += 1
        if d < 0: d += 2*y + 1
        else: x -= 1; d += 2*(y-x) + 1

# ── Filled circle ────────────────────────────────────────────
def _fcircle(cx, cy, r, c=1):
    for dy in range(-r, r+1):
        dx = int(math.sqrt(r*r - dy*dy))
        _hline(cx - dx, cy + dy, dx*2 + 1, c)

# ── Thick horizontal / vertical lines ───────────────────────
def _hline3(x, y, w, c=1):   # 3px tall
    _hline(x, y,   w, c)
    _hline(x, y+1, w, c)
    _hline(x, y+2, w, c)

# ══════════════════════════════════════════════════════════════
#  ICON LIBRARY  (pixel-art, drawn from scratch)
# ══════════════════════════════════════════════════════════════

# ── Large directional arrows (28×28 bounding box, cx/cy = centre) ──
def _icon_arrow_up(cx, cy):
    # Thick arrowhead pointing up
    for i in range(14):
        w = max(1, i * 2 - 1)
        _hline(cx - (i-1), cy - 13 + i, w)
    # Shaft
    _frect(cx - 4, cy + 1, 9, 10)

def _icon_arrow_down(cx, cy):
    _frect(cx - 4, cy - 11, 9, 10)
    for i in range(14):
        w = max(1, i * 2 - 1)
        _hline(cx - (i-1), cy + 3 - i, w)

def _icon_arrow_left(cx, cy):
    for i in range(14):
        h = max(1, i * 2 - 1)
        _vline(cx - 13 + i, cy - (i-1), h)
    _frect(cx + 1, cy - 4, 10, 9)

def _icon_arrow_right(cx, cy):
    _frect(cx - 11, cy - 4, 10, 9)
    for i in range(14):
        h = max(1, i * 2 - 1)
        _vline(cx + 3 - i, cy - (i-1), h)

def _icon_stop(cx, cy):
    # Octagon STOP shape
    _frect(cx - 9, cy - 5, 19, 11)
    _frect(cx - 5, cy - 9, 11, 19)
    for d in range(4):
        _frect(cx - 8 + d*6, cy - 8 + (d//2)*12 - (d%2)*0,  # corners
               3, 3 if d < 2 else 3)
    # Inner solid
    _frect(cx - 11, cy - 3, 23, 7)
    _frect(cx - 3,  cy - 11, 7, 23)

def _icon_stop(cx, cy):
    # Octagon (clipped square)
    pts = [
        (cx-11,cy-4,  23,9),
        (cx-9, cy-9,  19,19),
        (cx-4, cy-11, 9, 23),
    ]
    for x,y,w,h in pts: _frect(x, y, w, h)
    # STOP text in centre (3px tall mini-font approximation)
    # Draw "II" stop bars
    _frect(cx-5, cy-4, 4, 9)
    _frect(cx+1, cy-4, 4, 9)

# ── RC Car silhouette (for boot screen, 40×16) ───────────────
def _icon_car(x, y):
    # Body
    _frect(x+4, y+4, 32, 8)
    _frect(x+8, y,   22, 6)
    # Windscreen gap
    _frect(x+10, y+1, 8,  4, 0)
    _frect(x+20, y+1, 8,  4, 0)
    # Wheels (filled circles)
    _fcircle(x+8,  y+13, 4)
    _fcircle(x+32, y+13, 4)
    # Headlights
    _frect(x,    y+5, 4, 2)
    _frect(x+36, y+5, 4, 2)

# ── Checkmark ────────────────────────────────────────────────
def _icon_check(cx, cy):
    # Big thick checkmark
    coords = [
        (cx-10,cy+2),(cx-9,cy+3),(cx-8,cy+4),(cx-7,cy+5),(cx-6,cy+6),
        (cx-5, cy+5),(cx-4,cy+4),(cx-3,cy+3),(cx-2,cy+2),(cx-1,cy+1),
        (cx,   cy+0),(cx+1,cy-1),(cx+2,cy-2),(cx+3,cy-3),(cx+4,cy-4),
        (cx+5, cy-5),(cx+6,cy-6),(cx+7,cy-7),
    ]
    for px, py in coords:
        _frect(px, py, 2, 2)

# ── Warning triangle ─────────────────────────────────────────
def _icon_warning(cx, cy):
    for i in range(12):
        w = i * 2 + 1
        _hline(cx - i, cy - 5 + i, w)
    # Hollow inside
    for i in range(2, 10):
        w = max(0, i * 2 - 3)
        if w > 0: _hline(cx - (i-2), cy - 3 + i, w, 0)
    # Exclamation mark
    _frect(cx - 1, cy - 1, 3, 5)
    _frect(cx - 1, cy + 6, 3, 3)

# ── No-signal X ──────────────────────────────────────────────
def _icon_no_signal(cx, cy):
    r = 14
    _circle(cx, cy, r)
    _circle(cx, cy, r-1)
    for i in range(-9, 10):
        _px(cx + i, cy + i)
        _px(cx + i + 1, cy + i)
        _px(cx - i, cy + i)
        _px(cx - i + 1, cy + i)

# ── WiFi / signal bars ────────────────────────────────────────
def _icon_signal(x, y, strength=3):
    # strength 0–3
    heights = [3, 5, 7, 9]
    for i, h in enumerate(heights):
        c = 1 if i < strength else 0
        bx = x + i * 4
        by = y + (9 - h)
        _frect(bx, by, 3, h, 1)
        if c == 0:
            _rect(bx, by, 3, h, 1)
            _frect(bx+1, by+1, 1, h-2, 0)

# ── Tiny 5×3 digit font for the live angle readout ──────────
# Each digit: list of (x_offset, y_offset) pixels in a 5×5 grid
_MINI_DIGITS = {
    '0': [(0,0),(1,0),(2,0),(0,1),(2,1),(0,2),(2,2),(0,3),(2,3),(0,4),(1,4),(2,4)],
    '1': [(1,0),(1,1),(1,2),(1,3),(1,4)],
    '2': [(0,0),(1,0),(2,0),(2,1),(0,2),(1,2),(2,2),(0,3),(0,4),(1,4),(2,4)],
    '3': [(0,0),(1,0),(2,0),(2,1),(1,2),(2,2),(2,3),(0,4),(1,4),(2,4)],
    '4': [(0,0),(2,0),(0,1),(2,1),(0,2),(1,2),(2,2),(2,3),(2,4)],
    '5': [(0,0),(1,0),(2,0),(0,1),(0,2),(1,2),(2,2),(2,3),(0,4),(1,4),(2,4)],
    '6': [(0,0),(1,0),(0,1),(0,2),(1,2),(2,2),(0,3),(2,3),(0,4),(1,4),(2,4)],
    '7': [(0,0),(1,0),(2,0),(2,1),(2,2),(2,3),(2,4)],
    '8': [(0,0),(1,0),(2,0),(0,1),(2,1),(0,2),(1,2),(2,2),(0,3),(2,3),(0,4),(1,4),(2,4)],
    '9': [(0,0),(1,0),(2,0),(0,1),(2,1),(0,2),(1,2),(2,2),(2,3),(2,4)],
    '-': [(0,2),(1,2),(2,2)],
    '+': [(1,1),(0,2),(1,2),(2,2),(1,3)],
    '.': [(1,4)],
}

def _mini_text(s, x, y, c=1):
    """Draw string using 3×5 mini pixel font. Each char = 4px wide."""
    cx = x
    for ch in s:
        pts = _MINI_DIGITS.get(ch, [])
        for dx, dy in pts:
            _px(cx + dx, y + dy, c)
        cx += 4

def _mini_text_width(s):
    return len(s) * 4

# ══════════════════════════════════════════════════════════════
#  UI COMPONENT: TILT CROSSHAIR
#  Shows live pitch/roll as a dot on a 2D grid
# ══════════════════════════════════════════════════════════════
_CROSS_X  = 100   # centre X of crosshair widget
_CROSS_Y  = 36    # centre Y
_CROSS_R  = 14    # radius of outer circle
_CROSS_RANGE = 25.0  # ±degrees maps to ±radius

def _draw_crosshair(dp, dr):
    """
    Draw a circular crosshair with a moving dot.
    dp = pitch delta, dr = roll delta
    """
    cx, cy = _CROSS_X, _CROSS_Y
    # Outer ring
    _circle(cx, cy, _CROSS_R)
    # Centre crosshairs (dashed)
    _hline(cx - _CROSS_R + 2, cy, _CROSS_R - 3)
    _hline(cx + 3, cy, _CROSS_R - 3)
    _vline(cx, cy - _CROSS_R + 2, _CROSS_R - 3)
    _vline(cx, cy + 3, _CROSS_R - 3)
    # Dead-zone circle
    _circle(cx, cy, 3)
    # Dot position (clamp to circle)
    # roll → X,  pitch → Y (invert Y so forward = up)
    raw_dx = int(dr  / _CROSS_RANGE * _CROSS_R)
    raw_dy = int(-dp / _CROSS_RANGE * _CROSS_R)
    mag = math.sqrt(raw_dx*raw_dx + raw_dy*raw_dy)
    if mag > _CROSS_R - 2:
        scale = (_CROSS_R - 2) / mag
        raw_dx = int(raw_dx * scale)
        raw_dy = int(raw_dy * scale)
    dot_x = cx + raw_dx
    dot_y = cy + raw_dy
    _fcircle(dot_x, dot_y, 2)

# ══════════════════════════════════════════════════════════════
#  UI COMPONENT: ANGLE VALUE DISPLAY  (mini font)
# ══════════════════════════════════════════════════════════════
def _draw_angles(dp, dr):
    """Draw P/R angle values in mini font below crosshair."""
    y = _CROSS_Y + _CROSS_R + 3
    ps = "{:+.0f}".format(dp)
    rs = "{:+.0f}".format(dr)
    _mini_text("P" + ps, _CROSS_X - _CROSS_R, y)
    _mini_text("R" + rs, _CROSS_X + 2,         y)

# ══════════════════════════════════════════════════════════════
#  UI COMPONENT: LEFT PANEL  (command icon + label)
# ══════════════════════════════════════════════════════════════
_ICON_CX = 36   # icon centre X in left panel
_ICON_CY = 33   # icon centre Y

CMD_ICONS = {
    1: _icon_arrow_up,
    2: _icon_arrow_down,
    3: _icon_arrow_left,
    4: _icon_arrow_right,
    5: _icon_stop,
}

CMD_LABELS = {
    1: "FWD",
    2: "BCK",
    3: "LFT",
    4: "RGT",
    5: "STP",
}

def _draw_left_panel(cmd):
    """Draw current command icon and 3-char label."""
    fn = CMD_ICONS.get(cmd)
    if fn: fn(_ICON_CX, _ICON_CY)
    label = CMD_LABELS.get(cmd, "---")
    lx = _ICON_CX - len(label)*4 + 4
    _text(label, lx, 56)

# ══════════════════════════════════════════════════════════════
#  UI COMPONENT: TOP STATUS BAR
# ══════════════════════════════════════════════════════════════
def _draw_topbar(cmd, healthy, fail_count):
    # Solid top bar
    _frect(0, 0, 128, 11)
    # Command name (inverted = white bg, black text)
    label = {1:"FORWARD",2:"BACKWARD",3:"LEFT",4:"RIGHT",5:"STOP"}.get(cmd,"IDLE")
    lx = (80 - len(label)*8) // 2
    _text(label, lx, 2, 0)   # black text on white bar
    # Divider
    _vline(80, 0, 11, 0)
    # Signal in right section
    if healthy:
        _icon_signal(83, 1, 3)
    else:
        fc = min(fail_count, 3)
        _icon_signal(83, 1, 3 - fc)
    # Version dot
    _px(126, 5, 0)
    _px(127, 5, 0)

# ══════════════════════════════════════════════════════════════
#  UI COMPONENT: DIVIDERS
# ══════════════════════════════════════════════════════════════
def _draw_dividers():
    # Vertical divider between left panel and crosshair
    _vline(72, 11, 53)
    # Horizontal divider below top bar (already filled)

# ══════════════════════════════════════════════════════════════
#  SCREEN: BOOT
# ══════════════════════════════════════════════════════════════
def screen_boot(progress=0):
    """
    progress: 0–100
    Call repeatedly during init to animate the bar.
    """
    if not OLED_OK: return
    _oled.fill(0)

    # Title block (inverted)
    _frect(0, 0, 128, 14)
    _text("RC GESTURE", 16, 3, 0)

    # Car icon
    _icon_car(44, 20)

    # Version
    _text("v4.0", 48, 40)

    # Progress bar
    _rect(4, 52, 120, 8)
    bar_w = int(progress / 100.0 * 118)
    if bar_w > 0: _frect(5, 53, bar_w, 6)

    # Status text
    if progress < 40:
        msg = "IMU init..."
    elif progress < 70:
        msg = "ESP-NOW..."
    elif progress < 90:
        msg = "Warming up.."
    else:
        msg = "Ready!"
    _text(msg, (128 - len(msg)*8)//2, 54)

    _show()

# ══════════════════════════════════════════════════════════════
#  SCREEN: WAIT PIVOT
# ══════════════════════════════════════════════════════════════
def screen_wait_pivot(tick=0):
    """
    tick: incrementing counter used for pulsing animation.
    """
    if not OLED_OK: return
    _oled.fill(0)

    # Header
    _frect(0, 0, 128, 11)
    _text("CALIBRATE", 24, 2, 0)

    # Hand icon area (left side)
    # Draw simplified hand outline
    _rect(10, 18, 22, 28)
    _text("FLAT", 12, 24)
    _text("HAND", 12, 33)

    # Arrow pointing right toward button
    _hline(35, 32, 12)
    _frect(44, 29, 6, 7)   # arrowhead base
    for i in range(4):
        _hline(47+i, 31-i, 1)
        _hline(47+i, 33+i, 1)

    # Button box
    _rect(58, 24, 32, 18)
    _frect(59, 25, 30, 16)
    _text("BTN", 61, 27, 0)
    _text(" B ", 61, 36, 0)

    # Pulsing dot at bottom (blinks every ~500ms)
    pulse_phase = (tick // 12) % 4
    for i in range(4):
        bx = 48 + i * 8
        if i < pulse_phase:
            _fcircle(bx, 58, 2)
        else:
            _circle(bx, 58, 2)

    # Hint text
    _text("Hold still then press", 0, 57) if False else None  # too wide

    _show()

# ══════════════════════════════════════════════════════════════
#  SCREEN: PIVOT CONFIRMED
# ══════════════════════════════════════════════════════════════
def screen_pivot_set(pitch, roll):
    if not OLED_OK: return
    _oled.fill(0)

    # Header (inverted)
    _frect(0, 0, 128, 11)
    _text("PIVOT  LOCKED", 8, 2, 0)

    # Big checkmark
    _icon_check(30, 32)

    # Vertical divider
    _vline(60, 11, 53)

    # Values on right
    _text("PITCH", 65, 14)
    _frect(65, 23, 60, 10)
    ps = "{:+.1f}".format(pitch)
    _text(ps, 66, 25, 0)

    _text("ROLL", 65, 37)
    _frect(65, 46, 60, 10)
    rs = "{:+.1f}".format(roll)
    _text(rs, 66, 48, 0)

    _show()

# ══════════════════════════════════════════════════════════════
#  SCREEN: DRIVING  (main HUD)
# ══════════════════════════════════════════════════════════════
def screen_driving(cmd, dp, dr, healthy, fail_count=0):
    if not OLED_OK: return
    _oled.fill(0)

    # ── Top status bar ───────────────────────────────────────
    _draw_topbar(cmd, healthy, fail_count)

    # ── Divider ──────────────────────────────────────────────
    _draw_dividers()

    # ── Left: command icon ───────────────────────────────────
    _draw_left_panel(cmd)

    # ── Right: crosshair ────────────────────────────────────
    _draw_crosshair(dp, dr)

    # ── Right-bottom: live angle values ─────────────────────
    _draw_angles(dp, dr)

    _show()

# ══════════════════════════════════════════════════════════════
#  SCREEN: NO SIGNAL
# ══════════════════════════════════════════════════════════════
def screen_no_signal(tick=0):
    if not OLED_OK: return
    _oled.fill(0)

    # Header
    _frect(0, 0, 128, 11)
    _text("NO SIGNAL", 24, 2, 0)

    # Big no-signal icon, slightly animated (alternates every ~1s)
    if (tick // 25) % 2 == 0:
        _icon_no_signal(36, 37)
    else:
        _circle(36, 37, 14)
        _circle(36, 37, 13)

    # Right side: hint
    _vline(60, 11, 53)
    _text("Check", 65, 14)
    _text("car", 65, 23)
    _text("ESP32", 65, 32)
    _hline(63, 42, 63)
    _text("Retry", 65, 46)
    # Animated dots
    dots = "." * ((tick // 8) % 4)
    _text(dots, 105, 46)

    _show()

# ══════════════════════════════════════════════════════════════
#  SCREEN: IMU ERROR
# ══════════════════════════════════════════════════════════════
def screen_imu_error(retries):
    if not OLED_OK: return
    _oled.fill(0)

    _frect(0, 0, 128, 11)
    _text("IMU  ERROR", 20, 2, 0)

    _icon_warning(32, 37)

    _vline(60, 11, 53)
    _text("MPU-6050", 63, 14)
    _text("offline", 63, 23)
    _hline(63, 34, 63)
    _text("Retry", 63, 37)
    rc = "{}/{}".format(retries, IMU_RETRY_LIMIT)
    _text(rc, 63, 47)

    _show()

# ══════════════════════════════════════════════════════════════
#  MPU-6050
# ══════════════════════════════════════════════════════════════
MPU_ADDR     = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG  = 0x1B
ACCEL_XOUT_H = 0x3B
DLPF_CONFIG  = 0x1A
WHO_AM_I     = 0x75
ACCEL_SCALE  = 16384.0
GYRO_SCALE   = 65.5

def mpu_write(reg, val):
    i2c_mpu.writeto_mem(MPU_ADDR, reg, bytes([val]))

def mpu_whoami():
    try: return i2c_mpu.readfrom_mem(MPU_ADDR, WHO_AM_I, 1)[0] == 0x68
    except Exception: return False

def mpu_init():
    if not mpu_whoami(): raise RuntimeError("MPU-6050 not found")
    mpu_write(PWR_MGMT_1,   0x80); time.sleep_ms(100)
    mpu_write(PWR_MGMT_1,   0x01); time.sleep_ms(50)
    mpu_write(DLPF_CONFIG,  0x03)
    mpu_write(GYRO_CONFIG,  0x08)
    mpu_write(ACCEL_CONFIG, 0x00)
    time.sleep_ms(50); log("MPU-6050 OK")

def mpu_read_raw():
    data = i2c_mpu.readfrom_mem(MPU_ADDR, ACCEL_XOUT_H, 14)
    v = struct.unpack(">7h", data)
    return v[0], v[1], v[2], v[4], v[5], v[6]

# ══════════════════════════════════════════════════════════════
#  COMPLEMENTARY FILTER
# ══════════════════════════════════════════════════════════════
class ComplementaryFilter:
    __slots__ = ("pitch","roll","_last_t","_ready")
    def __init__(self): self.pitch=0.0; self.roll=0.0; self._last_t=0; self._ready=False
    def reset(self): self._last_t=time.ticks_ms(); self._ready=False
    def update(self, ax, ay, az, gx, gy):
        now = time.ticks_ms()
        ag=ax/ACCEL_SCALE; bg=ay/ACCEL_SCALE; cg=az/ACCEL_SCALE
        pa=math.atan2(bg, math.sqrt(ag*ag+cg*cg))*57.2958
        ra=math.atan2(-ag, math.sqrt(bg*bg+cg*cg))*57.2958
        if not self._ready:
            self.pitch=pa; self.roll=ra; self._last_t=now; self._ready=True
            return self.pitch, self.roll
        dt=time.ticks_diff(now,self._last_t)/1000.0; self._last_t=now
        if dt<=0.0 or dt>0.5: dt=LOOP_MS/1000.0
        self.pitch=ALPHA*(self.pitch+(gx/GYRO_SCALE)*dt)+(1-ALPHA)*pa
        self.roll =ALPHA*(self.roll +(gy/GYRO_SCALE)*dt)+(1-ALPHA)*ra
        return self.pitch, self.roll

# ══════════════════════════════════════════════════════════════
#  MEDIAN FILTER
# ══════════════════════════════════════════════════════════════
class MedianFilter:
    __slots__ = ("_buf","_idx","_size","_full")
    def __init__(self, size=GYRO_BUF_SIZE):
        self._size=size; self._buf=[0.0]*size; self._idx=0; self._full=False
    def update(self, val):
        self._buf[self._idx]=val; self._idx=(self._idx+1)%self._size
        if not self._full and self._idx==0: self._full=True
        w=self._buf if self._full else self._buf[:self._idx or self._size]
        s=sorted(w); m=len(s)>>1
        return s[m] if len(s)&1 else (s[m-1]+s[m])*0.5
    def reset(self): self._buf=[0.0]*self._size; self._idx=0; self._full=False

# ══════════════════════════════════════════════════════════════
#  GESTURE CLASSIFIER
# ══════════════════════════════════════════════════════════════
class GestureClassifier:
    __slots__ = ("_buf",)
    def __init__(self): self._buf=[]
    def update(self, dp, dr, gyro_mag):
        if gyro_mag>CIRCLE_GYRO_MAG: raw=5
        else:
            adp=abs(dp); adr=abs(dr)
            if adp<TILT_THRESH and adr<TILT_THRESH: raw=None
            elif adp>=adr: raw=1 if dp<0 else 2
            else: raw=3 if dr<0 else 4
        self._buf.append(raw)
        if len(self._buf)>CONFIRM_FRAMES: self._buf.pop(0)
        if len(self._buf)==CONFIRM_FRAMES:
            f=self._buf[0]
            if f is not None and all(g==f for g in self._buf): return f
        return None
    def reset(self): self._buf.clear()

# ══════════════════════════════════════════════════════════════
#  BUTTON
# ══════════════════════════════════════════════════════════════
class Button:
    __slots__ = ("_pin","_db","_prev","_t","just_pressed")
    def __init__(self, pin, debounce_ms=50):
        self._pin=pin; self._db=debounce_ms; self._prev=1; self._t=0; self.just_pressed=False
    def update(self):
        self.just_pressed=False; val=self._pin.value(); now=time.ticks_ms()
        if val!=self._prev and time.ticks_diff(now,self._t)>self._db:
            self._prev=val; self._t=now
            if val==0: self.just_pressed=True

# ══════════════════════════════════════════════════════════════
#  ESP-NOW SENDER
# ══════════════════════════════════════════════════════════════
class ESPNowSender:
    __slots__ = ("_en","_mac","_fail_count")
    def __init__(self, mac):
        self._mac=mac; self._fail_count=0; self._en=self._init()
    def _init(self):
        sta=network.WLAN(network.STA_IF); sta.active(True); sta.disconnect()
        en=espnow.ESPNow(); en.active(True); self._add_peer(en); return en
    def _add_peer(self, en):
        try: en.add_peer(self._mac)
        except Exception as e: log("add_peer: {}".format(e))
    def send(self, cmd):
        try:
            self._en.send(self._mac, bytes([cmd]), True)
            self._fail_count=0; return True
        except Exception as e:
            self._fail_count+=1; log("Send fail #{}: {}".format(self._fail_count,e))
            if self._fail_count==MAX_SEND_FAILS:
                log("Re-adding peer..."); self._add_peer(self._en)
            return False
    @property
    def healthy(self): return self._fail_count < MAX_SEND_FAILS

# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════
def main():
    try: wdt=WDT(timeout=WDT_TIMEOUT_MS); WDT_OK=True
    except Exception: wdt=None; WDT_OK=False

    def feed():
        if WDT_OK and wdt: wdt.feed()

    # ── Animated boot: IMU init (0–40%) ─────────────────────
    screen_boot(5)
    for attempt in range(1, 4):
        try: mpu_init(); break
        except Exception as e:
            log("MPU {}/3: {}".format(attempt, e))
            if attempt==3:
                screen_imu_error(attempt)
                while True: feed(); time.sleep_ms(1000)
            time.sleep_ms(500)
    screen_boot(40); feed()

    # ── ESP-NOW init (40–70%) ────────────────────────────────
    cf=ComplementaryFilter(); mf=MedianFilter()
    gc=GestureClassifier()
    screen_boot(55); feed()
    sender=ESPNowSender(RC_CAR_MAC)
    btn_b=Button(BTN_B)
    screen_boot(70); feed()

    # ── IMU warm-up (70–100%) ────────────────────────────────
    cf.reset()
    for i in range(20):
        try:
            ax,ay,az,gx,gy,gz=mpu_read_raw(); cf.update(ax,ay,az,gx,gy)
        except Exception: pass
        screen_boot(70 + i*1 + 1)
        feed(); time.sleep_ms(LOOP_MS)

    screen_boot(100); feed()
    time.sleep_ms(600)

    pivot_pitch=0.0; pivot_roll=0.0; pivot_set=False
    active_cmd=5; last_send_t=0; last_screen_t=0
    imu_errs=0; ui_tick=0

    screen_wait_pivot(0); log("Waiting for pivot – press BTN_B")

    # ════════════════════════════════════════════════════════
    #  MAIN LOOP
    # ════════════════════════════════════════════════════════
    while True:
        t0=time.ticks_ms(); feed()
        ui_tick += 1

        # ── IMU ──────────────────────────────────────────────
        try:
            ax,ay,az,gx,gy,gz=mpu_read_raw(); imu_errs=0
        except Exception as e:
            imu_errs+=1; log("IMU err #{}: {}".format(imu_errs,e))
            if imu_errs>=IMU_RETRY_LIMIT:
                screen_imu_error(imu_errs)
                try: mpu_init(); cf.reset(); mf.reset(); imu_errs=0
                except Exception as e2: log("Reinit fail: {}".format(e2))
            time.sleep_ms(LOOP_MS); continue

        pitch,roll=cf.update(ax,ay,az,gx,gy)
        gyro_mag=mf.update(math.sqrt(
            (gx/GYRO_SCALE)**2+(gy/GYRO_SCALE)**2+(gz/GYRO_SCALE)**2))

        # ── Button ───────────────────────────────────────────
        btn_b.update()
        if btn_b.just_pressed:
            log("Sampling pivot..."); sp=sr=0.0; cf.reset()
            for i in range(PIVOT_SAMPLES):
                try:
                    ax2,ay2,az2,gx2,gy2,gz2=mpu_read_raw()
                    p2,r2=cf.update(ax2,ay2,az2,gx2,gy2); sp+=p2; sr+=r2
                except Exception: pass
                feed(); time.sleep_ms(5)
            pivot_pitch=sp/PIVOT_SAMPLES; pivot_roll=sr/PIVOT_SAMPLES
            pivot_set=True; active_cmd=5
            cf.reset(); mf.reset(); gc.reset()
            log("Pivot P={:.2f} R={:.2f}".format(pivot_pitch,pivot_roll))
            screen_pivot_set(pivot_pitch,pivot_roll)
            time.sleep_ms(1200); feed(); continue

        # ── Waiting for pivot ────────────────────────────────
        if not pivot_set:
            now=time.ticks_ms()
            if time.ticks_diff(now,last_screen_t)>80:   # ~12fps animation
                screen_wait_pivot(ui_tick)
                last_screen_t=now
            e=time.ticks_diff(time.ticks_ms(),t0)
            if e<LOOP_MS: time.sleep_ms(LOOP_MS-e)
            continue

        # ── Gesture ──────────────────────────────────────────
        dp=pitch-pivot_pitch; dr=roll-pivot_roll
        cmd=gc.update(dp,dr,gyro_mag); now=time.ticks_ms()

        if cmd is not None and cmd!=active_cmd:
            active_cmd=cmd; sender.send(active_cmd); last_send_t=now
            log("CMD {} P{:+.1f} R{:+.1f} G{:.0f}".format(
                {1:"FWD",2:"BCK",3:"LFT",4:"RGT",5:"STP"}[active_cmd],
                dp,dr,gyro_mag))
        elif time.ticks_diff(now,last_send_t)>=HEARTBEAT_MS:
            sender.send(active_cmd); last_send_t=now

        # ── Display ──────────────────────────────────────────
        if time.ticks_diff(now,last_screen_t)>=OLED_REDRAW_MS:
            last_screen_t=now
            if not sender.healthy:
                screen_no_signal(ui_tick)
            else:
                screen_driving(active_cmd, dp, dr, sender.healthy,
                               sender._fail_count)

        e=time.ticks_diff(time.ticks_ms(),t0)
        if e<LOOP_MS: time.sleep_ms(LOOP_MS-e)

main()
