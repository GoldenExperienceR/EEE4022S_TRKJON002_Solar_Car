#!/usr/bin/env python3
"""Generate the solar-car KiCad project: symbol library + hierarchical schematic.
Files are written in KiCad 7 format (opens and upgrades cleanly in KiCad 8/9).
Edit PINMAP below and re-run to rewire the controllers."""
import uuid, os, collections

OUT = os.path.dirname(os.path.abspath(__file__))
PROJ = "solar_car"
LIB = "solar_car"
G = 2.54

# ---------------------------------------------------------------- PIN MAP ---
# ESP32 / Nano pin -> net.  (Pin names as they appear on the symbols.)
PINMAP = {
    "ESP32": {
        "GPIO25": "MOT_L_IN1", "GPIO26": "MOT_L_IN2",
        "GPIO27": "MOT_R_IN1", "GPIO14": "MOT_R_IN2",
        "GPIO21": "I2C_SDA",   "GPIO22": "I2C_SCL",
        "GPIO34": "SPD_L",     "GPIO35": "SPD_R",
        "GPIO5":  "US1_TRIG",  "GPIO18": "US1_ECHO",
        "GPIO17": "US2_TRIG",  "GPIO19": "US2_ECHO",
        "GPIO16": "US3_TRIG",  "GPIO23": "US3_ECHO",
        "GPIO4":  "US4_TRIG",  "GPIO13": "US4_ECHO",
        "GPIO32": "MOVING",
    },
    "NANO": {
        "A0": "LDR_TL", "A1": "LDR_TR", "A2": "LDR_BL", "A3": "LDR_BR",
        "D9": "SERVO_PAN", "D10": "SERVO_TILT",
        "D2": "MOVING",
    },
}
US_NAMES = {1: "Front-left (+30°)", 2: "Front-centre", 3: "Front-right (-30°)", 4: "Rear"}

POWER_NETS = ["GND", "+3V3", "+5V", "+VMOT", "VBAT", "VSYS"]

STUB = 5.08
def U(): return str(uuid.uuid4())
def f(v): return f"{v:.2f}".rstrip("0").rstrip(".") if v != int(v) else str(int(v))
FONT = "(effects (font (size 1.27 1.27)))"
FONT_HIDE = "(effects (font (size 1.27 1.27)) hide)"

# ------------------------------------------------------------- SYMBOLS ------
class Sym:
    def __init__(s, name, ref, desc, pins, graphics, ref_at, val_at, power=False):
        s.name, s.ref, s.desc, s.pins, s.graphics = name, ref, desc, pins, graphics
        s.ref_at, s.val_at, s.power = ref_at, val_at, power
        # pins: list of dict(num,name,type,x,y,angle,len,hide)

    def pin_by_name(s, nm):
        for p in s.pins:
            if p["name"] == nm: return p
        raise KeyError(f"{s.name}: no pin {nm}")

    def sexpr(s, libname=None):
        nm = f"{libname}:{s.name}" if libname else s.name
        head = f'(symbol "{nm}"'
        if s.power: head += " (power)"
        out = [head]
        if s.power: out.append("(pin_names (offset 0))")
        elif getattr(s, "small", False): out.append("(pin_numbers hide) (pin_names (offset 0) hide)")
        else: out.append("(pin_names (offset 1.016))")
        out.append("(in_bom yes) (on_board yes)")
        rh = FONT_HIDE if s.power else FONT
        out.append(f'(property "Reference" "{s.ref}" (at {f(s.ref_at[0])} {f(s.ref_at[1])} 0) {rh})')
        out.append(f'(property "Value" "{s.value_default}" (at {f(s.val_at[0])} {f(s.val_at[1])} 0) {FONT})')
        out.append(f'(property "Footprint" "" (at 0 0 0) {FONT_HIDE})')
        out.append(f'(property "Datasheet" "" (at 0 0 0) {FONT_HIDE})')
        out.append(f'(property "Description" "{s.desc}" (at 0 0 0) {FONT_HIDE})')
        out.append(f'(symbol "{s.name}_0_1" {" ".join(s.graphics)})')
        pins = []
        for p in s.pins:
            hide = " hide" if p.get("hide") else ""
            pins.append(f'(pin {p["type"]} line (at {f(p["x"])} {f(p["y"])} {p["angle"]}) (length {f(p["len"])}){hide} '
                        f'(name "{p["name"]}" {FONT}) (number "{p["num"]}" {FONT}))')
        out.append(f'(symbol "{s.name}_1_1" {" ".join(pins)})')
        out.append(")")
        return "\n  ".join(out)

def rect(x1, y1, x2, y2, fill="background"):
    return f"(rectangle (start {f(x1)} {f(y1)}) (end {f(x2)} {f(y2)}) (stroke (width 0.254) (type default)) (fill (type {fill})))"
def poly(pts, fill="none", w=0.254):
    p = " ".join(f"(xy {f(x)} {f(y)})" for x, y in pts)
    return f"(polyline (pts {p}) (stroke (width {w}) (type default)) (fill (type {fill})))"
def circ(x, y, r):
    return f"(circle (center {f(x)} {f(y)}) (radius {f(r)}) (stroke (width 0.254) (type default)) (fill (type background)))"
def txt(t, x, y, size=1.27):
    return f'(text "{t}" (at {f(x)} {f(y)} 0) (effects (font (size {size} {size}))))'

def box(name, ref, desc, left, right, halfw, value=None, top_text=None):
    """left/right: lists of (num, name) top-to-bottom; None = gap."""
    n = max(len(left), len(right))
    PP = 2 * G
    y0 = PP * ((n - 1) // 2)
    pins = []
    for side, lst in (("L", left), ("R", right)):
        for i, item in enumerate(lst):
            if item is None: continue
            num, pname = item
            x = -(halfw + G) if side == "L" else halfw + G
            pins.append(dict(num=str(num), name=pname, type="passive", x=x, y=y0 - i * PP,
                             angle=0 if side == "L" else 180, len=G))
    top, bot = y0 + G * 1.5, y0 - (n - 1) * PP - G * 1.5
    g = [rect(-halfw, top, halfw, bot)]
    if top_text: g.append(txt(top_text, 0, (top + bot) / 2, 1.0))
    s = Sym(name, ref, desc, pins, g, (-halfw, top + 1.27), (-halfw, bot - 1.27))
    s.value_default = value or name
    return s

def two_pin_v(name, ref, desc, graphics, n1="1", n2="2", nm1="~", nm2="~", value=None):
    pins = [dict(num="1", name=nm1, type="passive", x=0, y=5.08, angle=270, len=G),
            dict(num="2", name=nm2, type="passive", x=0, y=-5.08, angle=90, len=G)]
    s = Sym(name, ref, desc, pins, graphics, (3.81, 1.27), (3.81, -1.27))
    s.value_default = value or name; s.small = True
    return s

SYMS = {}
def add(s): SYMS[s.name] = s; return s

# ESP32 38-pin DevKit (SEEIT ESP32-DEV-38P / DevKitC layout)
add(box("ESP32_DEV_38P", "U", "ESP32-WROOM-32 38-pin dev board (SEEIT ESP32-DEV-38P), USB-C, CP2102",
    [(1, "3V3"), (2, "EN"), (3, "GPIO36"), (4, "GPIO39"), (5, "GPIO34"), (6, "GPIO35"), (7, "GPIO32"),
     (8, "GPIO33"), (9, "GPIO25"), (10, "GPIO26"), (11, "GPIO27"), (12, "GPIO14"), (13, "GPIO12"),
     (14, "GND"), (15, "GPIO13"), (16, "GPIO9"), (17, "GPIO10"), (18, "GPIO11"), (19, "5V")],
    [(38, "GND"), (37, "GPIO23"), (36, "GPIO22"), (35, "GPIO1/TX"), (34, "GPIO3/RX"), (33, "GPIO21"),
     (32, "GND"), (31, "GPIO19"), (30, "GPIO18"), (29, "GPIO5"), (28, "GPIO17"), (27, "GPIO16"),
     (26, "GPIO4"), (25, "GPIO0"), (24, "GPIO2"), (23, "GPIO15"), (22, "GPIO8"), (21, "GPIO7"), (20, "GPIO6")],
    12.7, top_text=None))
add(box("Arduino_Nano", "U", "Arduino Nano (ATmega328P) - solar tracker controller",
    [(1, "D1/TX"), (2, "D0/RX"), (3, "RST"), (4, "GND"), (5, "D2"), (6, "D3"), (7, "D4"), (8, "D5"),
     (9, "D6"), (10, "D7"), (11, "D8"), (12, "D9"), (13, "D10"), (14, "D11"), (15, "D12")],
    [(30, "VIN"), (29, "GND"), (28, "RST"), (27, "5V"), (26, "A7"), (25, "A6"), (24, "A5"), (23, "A4"),
     (22, "A3"), (21, "A2"), (20, "A1"), (19, "A0"), (18, "AREF"), (17, "3V3"), (16, "D13")], 10.16))
add(box("DRV8871_Module", "U", "DRV8871 brushed DC motor driver breakout (6.5-45 V, 3.6 A peak)",
    [(1, "VM"), (2, "GND"), (3, "IN1"), (4, "IN2")], [(5, "OUT1"), (6, "OUT2")], 7.62))
add(box("XL4015_Module", "U", "XL4015 5 A adjustable buck converter module",
    [(1, "IN+"), (2, "IN-")], [(3, "OUT+"), (4, "OUT-")], 7.62))
add(box("CN3303_Charger", "U", "CN3303 3S Li-ion charger/boost module, USB input on board",
    [(1, "USB_5V"), (2, "USB_GND")], [(3, "BAT+"), (4, "BAT-")], 7.62))
add(box("MPPT_V08A", "U", "MPPT-V08A 5 A solar charge controller (set to 12.6 V CV for 3S)",
    [(1, "PV+"), (2, "PV-")], [(3, "BAT+"), (4, "BAT-")], 7.62))
add(box("INA219_Module", "U", "INA219 I2C high-side current/voltage monitor breakout (0.1 R shunt)",
    [(1, "VCC"), (2, "GND"), (3, "SCL"), (4, "SDA")], [(5, "VIN+"), (6, "VIN-")], 7.62))
add(box("BMS_3S_Balance", "U", "HKD 3S 10A balance board (balancing only - no current/voltage cut-off)",
    [(1, "B-"), (2, "B1"), (3, "B2"), (4, "B+")], [], 6.35))
add(box("Battery_3S_18650", "BT", "3x 18650 holder in series (Samsung INR18650-35E) - taps for balancing",
    [], [(1, "BAT+"), (2, "TAP2"), (3, "TAP1"), (4, "BAT-")], 7.62))
add(box("HC-SR04", "U", "HC-SR04 ultrasonic distance sensor (5 V)",
    [], [(1, "VCC"), (2, "TRIG"), (3, "ECHO"), (4, "GND")], 6.35))
add(box("Photo_Interrupter", "U", "Slotted photo-interrupter speed sensor module (LM393 comparator)",
    [], [(1, "VCC"), (2, "GND"), (3, "DO")], 6.35))
add(box("Servo_MG90S", "M", "MG90S metal-gear micro servo",
    [(1, "PWM"), (2, "+5V"), (3, "GND")], [], 6.35))
# two-pin parts
add(two_pin_v("R", "R", "Resistor", [rect(-1.016, 2.54, 1.016, -2.54, "none")]))
add(two_pin_v("LDR", "R", "Light dependent resistor",
    [rect(-1.016, 2.54, 1.016, -2.54, "none"),
     poly([(-4.064, 3.302), (-1.778, 1.016)]), poly([(-1.778, 1.016), (-2.54, 1.27), (-2.032, 1.778), (-1.778, 1.016)], "outline", 0.1),
     poly([(-4.064, 1.27), (-1.778, -1.016)]), poly([(-1.778, -1.016), (-2.54, -0.762), (-2.032, -0.254), (-1.778, -1.016)], "outline", 0.1)]))
add(two_pin_v("Motor_DC", "M", "AC032 TT geared DC motor 1:48",
    [circ(0, 0, 2.794), txt("M", 0, 0, 1.778)], nm1="+", nm2="-"))
add(two_pin_v("Solar_Panel", "SC", "4.2 W 9 V solar panel 165x165",
    [rect(-2.54, 2.54, 2.54, -2.54), poly([(-2.54, 0), (2.54, 0)], w=0.15), poly([(0, 2.54), (0, -2.54)], w=0.15),
     txt("+", 1.6, 3.4, 1.0)], nm1="+", nm2="-"))
# Schottky diode, horizontal: A left, K right
d = Sym("D_Schottky", "D", "Schottky diode (1N5822, 3 A 40 V)",
        [dict(num="2", name="A", type="passive", x=-5.08, y=0, angle=0, len=G),
         dict(num="1", name="K", type="passive", x=5.08, y=0, angle=180, len=G)],
        [poly([(-1.27, 1.27), (-1.27, -1.27), (1.27, 0), (-1.27, 1.27)], "background"),
         poly([(1.778, 1.524), (1.778, 1.27), (1.27, 1.27), (1.27, -1.27), (0.762, -1.27), (0.762, -1.524)])],
        (0, 2.54), (0, -2.54)); d.value_default = "1N5822"
d.small = True; add(d)
sw = Sym("SW_SPDT_Rocker", "SW", "KCD1-101 rocker switch (main power)",
         [dict(num="2", name="COM", type="passive", x=-5.08, y=0, angle=0, len=G),
          dict(num="1", name="A", type="passive", x=5.08, y=G, angle=180, len=G),
          dict(num="3", name="B", type="passive", x=5.08, y=-G, angle=180, len=G)],
         [circ(-2.032, 0, 0.508), circ(2.032, G, 0.508), circ(2.032, -G, 0.508),
          poly([(-1.524, 0.254), (2.032, 1.905)])], (-2.54, 5.08), (-2.54, -5.08)); sw.value_default = "KCD1-101"; sw.small = True; add(sw)

def power_sym(net):
    if net == "GND":
        g = [poly([(0, 0), (0, -1.27), (1.27, -1.27), (0, -2.54), (-1.27, -1.27), (0, -1.27)])]
        val = (0, -3.81); ang = 270
    else:
        g = [poly([(-0.762, 1.27), (0, 2.54)]), poly([(0, 0), (0, 2.54)]), poly([(0, 2.54), (0.762, 1.27)])]
        val = (0, 3.556); ang = 90
    nm = "PWR_" + net.replace("+", "P")
    s = Sym(nm, "#PWR", f"Power net {net}",
            [dict(num="1", name=net, type="power_in", x=0, y=0, angle=ang, len=0, hide=True)],
            g, (0, -6.35), val, power=True)
    s.value_default = net
    return s
for n in POWER_NETS: add(power_sym(n))
flg = Sym("PWR_FLAG", "#FLG", "Marks a net as driven (for ERC)",
          [dict(num="1", name="pwr", type="power_out", x=0, y=0, angle=90, len=0, hide=True)],
          [poly([(0, 0), (0, 1.27), (-1.016, 1.905), (0, 2.54), (1.016, 1.905), (0, 1.27)])],
          (0, 1.905), (0, 3.81), power=True)
flg.value_default = "PWR_FLAG"; add(flg)

def write_lib():
    body = "\n".join("  " + s.sexpr() for s in SYMS.values())
    with open(os.path.join(OUT, f"{LIB}.kicad_sym"), "w") as fh:
        fh.write(f'(kicad_symbol_lib (version 20220914) (generator {PROJ}_gen)\n{body}\n)\n')

# ------------------------------------------------------------ SCHEMATIC -----
class Sheet:
    def __init__(s, file, title, page):
        s.file, s.title, s.page = file, title, page
        s.uuid = U(); s.items = []; s.parts = []; s.used = set(); s.texts = []
    def place(s, sym, ref, x, y, conns, value=None, note=None):
        s.parts.append(dict(sym=SYMS[sym], ref=ref, x=x, y=y, conns=conns, value=value, note=note))
        s.used.add(sym)
    def text(s, t, x, y, size=1.27):
        s.texts.append((t, x, y, size))

SHEETS = []
def sheet(*a): sh = Sheet(*a); SHEETS.append(sh); return sh

# ---- Power subsystem ----
P = sheet("power.kicad_sch", "Power subsystem - generation, charging, storage, switching", 2)
P.place("Solar_Panel", "SC1", 35, 55, {"+": "PV_P", "-": "PV_MID"}, "4.2 W 9 V")
P.place("Solar_Panel", "SC2", 35, 95, {"+": "PV_MID", "-": "PV_N"}, "4.2 W 9 V")
P.place("MPPT_V08A", "U8", 100, 70, {"PV+": "PV_P", "PV-": "PV_N", "BAT+": "MPPT_OUT", "BAT-": "GND"}, "MPPT-V08A")
P.place("INA219_Module", "U9", 175, 70, {"VCC": "+3V3", "GND": "GND", "SCL": "I2C_SCL", "SDA": "I2C_SDA",
                                         "VIN+": "MPPT_OUT", "VIN-": "CHG_MPPT"}, "INA219 solar charge (0x41, A0 bridged)")
P.place("D_Schottky", "D1", 245, 70, {"A": "CHG_MPPT", "K": "VBAT"})
P.place("CN3303_Charger", "U7", 100, 145, {"BAT+": "CHG_USB", "BAT-": "GND"}, "CN3303 USB 3S charger")
P.place("D_Schottky", "D2", 245, 145, {"A": "CHG_USB", "K": "VBAT"})
P.place("Battery_3S_18650", "BT1", 175, 200, {"BAT+": "VBAT", "TAP2": "CELL2", "TAP1": "CELL1", "BAT-": "GND"}, "3S 18650 (INR18650-35E)")
P.place("BMS_3S_Balance", "U11", 250, 200, {"B-": "GND", "B1": "CELL1", "B2": "CELL2", "B+": "VBAT"}, "HKD 3S 10A balance")
P.place("SW_SPDT_Rocker", "SW1", 300, 110, {"COM": "VBAT", "A": "SW_OUT"})
P.place("INA219_Module", "U10", 360, 110, {"VCC": "+3V3", "GND": "GND", "SCL": "I2C_SCL", "SDA": "I2C_SDA",
                                           "VIN+": "SW_OUT", "VIN-": "VSYS"}, "INA219 system load (0x40)")
P.text("U9 measures MPPT charge output; U10 measures total load after SW1.\\n"
       "Chargers sit before SW1, so the pack charges with the car switched off.\\n"
       "D1/D2 drop ~0.4 V: 12.6 V CV chargers stop near 12.2 V (~90 % SOC).\\n"
       "U11 balances only - no over-current/over-discharge cut-off. No fuse fitted.\\n"
       "PV- is common with BAT- inside the MPPT (buck topology) - verify on your module.", 30, 238)

# ---- Controller + driver ----
C = sheet("control_drive.kicad_sch", "Controller and driver - ESP32, bucks, motor drive, sensors", 3)
esp = {"3V3": "+3V3", "5V": "+5V", "GND": "GND"}
esp.update(PINMAP["ESP32"])
C.place("ESP32_DEV_38P", "U1", 205, 115, esp, "ESP32-DEV-38P")
for i in range(1, 5):
    y = 35 + (i - 1) * 50
    C.place("HC-SR04", f"U{11+i}", 35, y, {"VCC": "+5V", "TRIG": f"US{i}_TRIG", "ECHO": f"US{i}_ECHO5V", "GND": "GND"},
            f"HC-SR04 {US_NAMES[i]}")
    C.place("R", f"R{2*i-1}", 100, y + 5, {"#1": f"US{i}_ECHO5V", "#2": f"US{i}_ECHO"}, "2.2k")
    C.place("R", f"R{2*i}", 125, y + 5, {"#1": f"US{i}_ECHO", "#2": "GND"}, "4.7k")
C.place("XL4015_Module", "U5", 315, 30, {"IN+": "VSYS", "IN-": "GND", "OUT+": "+5V", "OUT-": "GND"}, "XL4015 -> 5.0 V logic")
C.place("XL4015_Module", "U6", 315, 65, {"IN+": "VSYS", "IN-": "GND", "OUT+": "+VMOT", "OUT-": "GND"}, "XL4015 -> 7.5 V motors")
C.place("DRV8871_Module", "U3", 300, 115, {"VM": "+VMOT", "GND": "GND", "IN1": "MOT_L_IN1", "IN2": "MOT_L_IN2",
                                          "OUT1": "ML_A", "OUT2": "ML_B"}, "DRV8871 left")
C.place("Motor_DC", "M1", 355, 115, {"+": "ML_A", "-": "ML_B"}, "TT front-left")
C.place("Motor_DC", "M2", 385, 115, {"+": "ML_A", "-": "ML_B"}, "TT rear-left")
C.place("DRV8871_Module", "U4", 300, 175, {"VM": "+VMOT", "GND": "GND", "IN1": "MOT_R_IN1", "IN2": "MOT_R_IN2",
                                          "OUT1": "MR_A", "OUT2": "MR_B"}, "DRV8871 right")
C.place("Motor_DC", "M3", 355, 175, {"+": "MR_A", "-": "MR_B"}, "TT front-right")
C.place("Motor_DC", "M4", 385, 175, {"+": "MR_A", "-": "MR_B"}, "TT rear-right")
C.place("Photo_Interrupter", "U16", 175, 205, {"VCC": "+3V3", "GND": "GND", "DO": "SPD_L"}, "Encoder left")
C.place("Photo_Interrupter", "U17", 235, 205, {"VCC": "+3V3", "GND": "GND", "DO": "SPD_R"}, "Encoder right")
C.text("ECHO dividers: 2.2k series / 4.7k to GND -> 5 V echo becomes 3.4 V.\\n"
       "Each side's two motors run in parallel on one DRV8871 (skid-steer); swap leads on a mirrored motor.\\n"
       "Encoders run from 3V3 so DO is safe for ESP32 inputs (GPIO34/35 are input-only, no pull-ups).", 30, 245)

# ---- Solar tracker ----
S = sheet("tracker.kicad_sch", "Solar tracker - Nano, LDRs, servos", 4)
nano = {"5V": "+5V", "GND": "GND"}
nano.update(PINMAP["NANO"])
S.place("Arduino_Nano", "U2", 230, 120, nano, "Arduino Nano")
for i, (net, nm) in enumerate([("LDR_TL", "top-left"), ("LDR_TR", "top-right"), ("LDR_BL", "bottom-left"), ("LDR_BR", "bottom-right")]):
    x = 45 + i * 35
    S.place("LDR", f"R{9+i}", x, 100, {"#1": "+5V", "#2": net}, f"LDR {nm}")
    S.place("R", f"R{13+i}", x, 135, {"#1": net, "#2": "GND"}, "10k")
S.place("Servo_MG90S", "M5", 340, 100, {"PWM": "SERVO_PAN", "+5V": "+5V", "GND": "GND"}, "MG90S pan")
S.place("Servo_MG90S", "M6", 340, 145, {"PWM": "SERVO_TILT", "+5V": "+5V", "GND": "GND"}, "MG90S tilt")
S.text("MOVING: ESP32 drives high while the car moves; Nano then parks the panel flat.\\n"
       "Nano + servos fed from the 5 V logic buck into the Nano 5V pin (not VIN).", 30, 235)

# ------------------------------------------------------------- emit -----
def outward(p):
    return {0: (-1, 0), 180: (1, 0), 270: (0, -1), 90: (0, 1)}[p["angle"]]

def label_angle(dx, dy):
    return {(-1, 0): 180, (1, 0): 0, (0, -1): 90, (0, 1): 270}[(dx, dy)]

net_sheets = collections.defaultdict(set)
for sh in SHEETS:
    for p in sh.parts:
        for net in p["conns"].values():
            if net: net_sheets[net].add(sh.file)

ROOT_UUID = U()
pwr_count = [0]

def sym_instance(sym, ref, x, y, value, path, lib_id=None, props_extra=""):
    lib_id = lib_id or f"{LIB}:{sym.name}"
    val = value or sym.value_default
    ref_hide = " hide" if sym.power else ""
    o = [f'(symbol (lib_id "{lib_id}") (at {f(x)} {f(y)} 0) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid {U()})',
         f'(property "Reference" "{ref}" (at {f(x+sym.ref_at[0])} {f(y-sym.ref_at[1])} 0) (effects (font (size 1.27 1.27)){" (justify left)" if not sym.power else ""}{ref_hide}))',
         f'(property "Value" "{val}" (at {f(x+sym.val_at[0])} {f(y-sym.val_at[1])} 0) (effects (font (size 1.27 1.27)){" (justify left)" if not sym.power else ""}))',
         f'(property "Footprint" "" (at {f(x)} {f(y)} 0) {FONT_HIDE})',
         f'(property "Datasheet" "" (at {f(x)} {f(y)} 0) {FONT_HIDE})']
    for p in sym.pins: o.append(f'(pin "{p["num"]}" (uuid {U()}))')
    o.append(f'(instances (project "{PROJ}" (path "{path}" (reference "{ref}") (unit 1))))')
    o.append(")")
    return "\n  ".join(o)

ROT_SUP = {(0, -1): 0, (-1, 0): 90, (0, 1): 180, (1, 0): 270}
ROT_GND = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}
def rotv(x, y, rot):
    for _ in range(rot // 90): x, y = y, -x
    return x, y
def power_instance(sym, ref, x, y, rot, path):
    vx, vy = rotv(sym.val_at[0], -sym.val_at[1], rot)
    right = vx > 0
    if rot == 90: right = not right  # KiCad mirrors justification for this orientation
    just = "" if vx == 0 else (" (justify left)" if right else " (justify right)")
    o = [f'(symbol (lib_id "{LIB}:{sym.name}") (at {f(x)} {f(y)} {rot}) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid {U()})',
         f'(property "Reference" "{ref}" (at {f(x)} {f(y)} 0) {FONT_HIDE})',
         f'(property "Value" "{sym.value_default}" (at {f(x+vx)} {f(y+vy)} {rot % 180}) (effects (font (size 1.27 1.27)){just}))',
         f'(property "Footprint" "" (at {f(x)} {f(y)} 0) {FONT_HIDE})',
         f'(property "Datasheet" "" (at {f(x)} {f(y)} 0) {FONT_HIDE})',
         f'(pin "1" (uuid {U()}))',
         f'(instances (project "{PROJ}" (path "{path}" (reference "{ref}") (unit 1))))', ")"]
    return "\n  ".join(o)

def emit_sheet(sh, path):
    items = []
    used = set(sh.used)
    for part in sh.parts:
        sym, x, y = part["sym"], part["x"], part["y"]
        items.append(sym_instance(sym, part["ref"], x, y, part["value"], path))
        conns = part["conns"]
        for p in sym.pins:
            key = p["name"]
            net = conns.get("#" + p["num"]) if ("#" + p["num"]) in conns else conns.get(key)
            if key in ("~",) and ("#" + p["num"]) not in conns: net = None
            px, py = x + p["x"], y - p["y"]
            if not net:
                if sym.power: continue
                items.append(f'(no_connect (at {f(px)} {f(py)}) (uuid {U()}))'); continue
            dx, dy = outward(p)
            ex, ey = px + dx * STUB, py + dy * STUB
            items.append(f'(wire (pts (xy {f(px)} {f(py)}) (xy {f(ex)} {f(ey)})) (stroke (width 0) (type default)) (uuid {U()}))')
            if net in POWER_NETS:
                ps = SYMS["PWR_" + net.replace("+", "P")]; used.add(ps.name)
                pwr_count[0] += 1
                rot = (ROT_GND if net == "GND" else ROT_SUP)[(dx, dy)]
                items.append(power_instance(ps, f"#PWR{pwr_count[0]:03d}", ex, ey, rot, path))
            else:
                a = label_angle(dx, dy)
                if dx == 0: a = 0  # keep text horizontal on vertical stubs
                just = "left" if a in (0, 90) else "right"
                if len(net_sheets[net]) > 1:
                    items.append(f'(global_label "{net}" (shape passive) (at {f(ex)} {f(ey)} {a}) (fields_autoplaced) '
                                 f'(effects (font (size 1.27 1.27)) (justify {just})) (uuid {U()}) '
                                 f'(property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {f(ex)} {f(ey)} 0) {FONT_HIDE}))')
                else:
                    items.append(f'(label "{net}" (at {f(ex)} {f(ey)} {a}) (fields_autoplaced) '
                                 f'(effects (font (size 1.27 1.27)) (justify {just} bottom)) (uuid {U()}))')
    for t, x, y, size in sh.texts:
        items.append(f'(text "{t}" (at {f(x)} {f(y)} 0) (effects (font (size {size} {size})) (justify left bottom)) (uuid {U()}))')
    return items, used

def lib_symbols(names):
    return "(lib_symbols\n  " + "\n  ".join(SYMS[n].sexpr(LIB) for n in sorted(names)) + "\n)"

def title_block(t, page):
    return (f'(title_block (title "{t}") (date "2026-10-06") (rev "2") '
            f'(company "EEE4022S Solar Car - Yonatan (TRKJON002)") (comment 1 "Module-level wiring; nets joined by labels"))')

def write_sheet(sh):
    path = f"/{ROOT_UUID}/{sh.uuid}"
    items, used = emit_sheet(sh, path)
    # PWR_FLAGs on the power sheet
    if sh is P:
        x0 = 40
        for i, net in enumerate(POWER_NETS + []):
            x = x0 + i * 20; y = 265
            ps = SYMS["PWR_" + net.replace("+", "P")]; used.add(ps.name); used.add("PWR_FLAG")
            pwr_count[0] += 1
            items.append(f'(wire (pts (xy {f(x)} {f(y)}) (xy {f(x)} {f(y+2.54)})) (stroke (width 0) (type default)) (uuid {U()}))')
            # supplies: symbol on top, flag hanging below; GND: flag on top, symbol below
            top, bot = (SYMS["PWR_FLAG"], ps) if net == "GND" else (ps, SYMS["PWR_FLAG"])
            for s_, yy in ((top, y), (bot, y + 2.54)):
                r = f"#FLG{i+1:02d}" if s_.name == "PWR_FLAG" else f"#PWR{pwr_count[0]:03d}"
                ang = 180 if (s_.name == "PWR_FLAG" and net != "GND") else 0
                inst = sym_instance(s_, r, x, yy, None, path).replace(f"(at {f(x)} {f(yy)} 0)", f"(at {f(x)} {f(yy)} {ang})", 1)
                if s_.name == "PWR_FLAG":  # hide the flag's own text so it doesn't sit on the net name
                    inst = inst.replace('"PWR_FLAG" (at', '"PWR_FLAG" (at', 1)
                    i0 = inst.index('(property "Value"'); i1 = inst.index("\n", i0)
                    inst = inst[:i0] + inst[i0:i1].replace("(font (size 1.27 1.27)))", "(font (size 1.27 1.27)) hide)") + inst[i1:]
                items.append(inst)
        items.append(f'(text "ERC power flags" (at {f(x0)} 256 0) (effects (font (size 1.27 1.27)) (justify left bottom)) (uuid {U()}))')
    body = "\n".join(items)
    with open(os.path.join(OUT, sh.file), "w") as fh:
        fh.write(f'(kicad_sch (version 20230121) (generator eeschema)\n(uuid {sh.uuid})\n(paper "A3")\n'
                 f'{title_block(sh.title, sh.page)}\n{lib_symbols(used)}\n{body}\n)\n')

def write_root():
    items = []
    w, h = 70, 35
    for i, sh in enumerate(SHEETS):
        x = 30 + i * 85; y = 70
        items.append(f'(sheet (at {x} {y}) (size {w} {h}) (fields_autoplaced) (stroke (width 0.1524) (type solid)) '
                     f'(fill (color 0 0 0 0.0000)) (uuid {sh.uuid})\n'
                     f'  (property "Sheetname" "{sh.title.split(" - ")[0]}" (at {x} {f(y-0.7)} 0) (effects (font (size 1.27 1.27)) (justify left bottom)))\n'
                     f'  (property "Sheetfile" "{sh.file}" (at {x} {f(y+h+0.6)} 0) (effects (font (size 1.27 1.27)) (justify left top)))\n'
                     f'  (instances (project "{PROJ}" (path "/{ROOT_UUID}" (page "{sh.page}")))))')
    items.append(f'(text "Solar car - system schematic\\n'
                 f'Power: 3S Li-ion, MPPT solar + USB charging, 5 V logic and 7.5 V motor buck rails.\\n'
                 f'Nets cross sheets via global labels; GND/+3V3/+5V/+VMOT/VBAT/VSYS are power symbols." '
                 f'(at 40 40 0) (effects (font (size 1.6 1.6)) (justify left bottom)) (uuid {U()}))')
    with open(os.path.join(OUT, f"{PROJ}.kicad_sch"), "w") as fh:
        fh.write(f'(kicad_sch (version 20230121) (generator eeschema)\n(uuid {ROOT_UUID})\n(paper "A4")\n'
                 f'{title_block("Solar car - top level", 1)}\n(lib_symbols)\n' + "\n".join(items) +
                 '\n(sheet_instances (path "/" (page "1")))\n)\n')

def write_project():
    with open(os.path.join(OUT, f"{PROJ}.kicad_pro"), "w") as fh:
        fh.write('{\n  "meta": {"filename": "%s.kicad_pro", "version": 1},\n  "schematic": {},\n  "boards": [],\n  "sheets": [],\n  "text_variables": {}\n}\n' % PROJ)
    with open(os.path.join(OUT, "sym-lib-table"), "w") as fh:
        fh.write(f'(sym_lib_table\n  (lib (name "{LIB}")(type "KiCad")(uri "${{KIPRJMOD}}/{LIB}.kicad_sym")(options "")(descr "Solar car modules"))\n)\n')

if __name__ == "__main__":
    write_lib()
    for sh in SHEETS: write_sheet(sh)
    write_root(); write_project()
    print("ok")
