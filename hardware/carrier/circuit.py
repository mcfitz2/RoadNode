"""RoadNode OBD carrier board: circuit description.

Generates carrier.net with SKiDL (SKiDL's schematic generator cannot place and
route a circuit this size, so the netlist is the source of truth). Run with the venv in
this directory (KiCad's bundled Python + skidl):

    .venv/bin/python circuit.py

Buck values are reasoned in notes/buck.md; the comparator thresholds come from
sim/wake_thr.py and sim/uvlo_thr.py.
"""
import os

import builtins

from skidl import KICAD9, ERC, Net, Part, generate_netlist, lib_search_paths, set_default_tool

HERE = os.path.dirname(os.path.abspath(__file__))
set_default_tool(KICAD9)
lib_search_paths[KICAD9] += ["/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols", os.path.join(HERE, "lib")]

R0603 = "Resistor_SMD:R_0603_1608Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
RN = "rn_parts"


def part(lib, sym, value, footprint, lcsc, prefix=None, ref=None):
    """Instantiates a part; the LCSC field feeds the JLC BOM ("" = hand-fit, not assembled)."""
    p = Part(lib, sym, value=value, footprint=footprint, ref_prefix=prefix, ref=ref)
    p.fields["LCSC"] = lcsc
    p.fixed_ref = bool(ref)
    return p


def R(value, lcsc, fp=R0603):
    return part("Device", "R", value, fp, lcsc)


def C(value, lcsc, fp=C0603):
    return part("Device", "C", value, fp, lcsc)


def ic(sym, value, fp, lcsc, prefix="U", ref=None):
    return part(RN, sym, value, f"{RN}:{fp}", lcsc, prefix, ref)


# Generic Basic-library passives
R_LCSC = {"1k": "C21190", "10k": "C25804", "100k": "C25803", "1M": "C22935", "120": "C22787", "100": "C22775"}
C100N = "C14663"  # 100 nF 50 V X7R 0603
C1U = "C15849"  # 1 uF 50 V 0603

gnd = Net("GND")
gnd.drive = 3  # POWER

# ---------------------------------------------------------------- nets
v_car = Net("V_CAR")  # OBD pin 16 via pigtail, unfused
vin = Net("VIN")  # fused, clamped
v5r = Net("V5R")  # buck output, on unless the low-voltage cutoff holds the buck off
h5v = Net("H_5V")  # Heltec header 5V pin (= its USB VBUS net)
v3a = Net("V3A")  # always-on 3.3 V from VIN: comparators and pull-ups, up while the buck is off
vref = Net("VREF")  # 2.5 V precision reference for both comparator thresholds
v3v3 = Net("3V3")  # Heltec always-on 3.3 V (header)
vext = Net("VEXT")  # Heltec switched 3.3 V (GPIO36 low = on)
canh, canl = Net("CANH"), Net("CANL")
for n in (vin, v5r, v3v3, vext):
    n.drive = 3

# ---------------------------------------------------------------- vehicle connector
# JST XH 4-pin, pigtail from an OBD-II cable: 16 -> 1, 4/5 -> 2, 6 -> 3, 14 -> 4
j_obd = ic("B4B-XH-A", "OBD pigtail", "CONN-TH_B4B-XH-A-LF-SN", "C144395", ref="J1")
j_obd[1] += v_car
j_obd[2] += gnd
j_obd[3] += canh
j_obd[4] += canl

# ---------------------------------------------------------------- input protection
f_in = ic("MSMD150-33V", "1.5A hold", "F1812", "C2937110", "F")
f_in[1] += v_car
f_in[2] += vin
tvs = ic("SMBJ26A_C123820", "SMBJ26A", "SMB_L4.6-W3.6-LS5.3-RD", "C19077580", "D")
tvs["C"] += vin  # also crowbars a reversed pigtail until the PTC trips
tvs["A"] += gnd
c_bulk = part("Device", "C_Polarized", "47u 50V", "Capacitor_SMD:CP_Elec_6.3x7.7", "C970679")
c_bulk[1] += vin
c_bulk[2] += gnd
c_in1 = C("10u 50V", "C13585", "Capacitor_SMD:C_1206_3216Metric")  # 50 V > the 42 V TVS clamp
c_in2 = C("100n 100V", "C28233", "Capacitor_SMD:C_0805_2012Metric")
for c in (c_in1, c_in2):
    c[1] += vin
    c[2] += gnd

# ---------------------------------------------------------------- buck, 5.2 V (notes/buck.md)
u_buck = ic("LMR36520ADDAR", "LMR36520ADDAR", "POWERPAD-SO-8_L4.9-W3.9-P1.27-LS6.0-BL-EP", "C2879422")
sw, fb, en, boot, vcc_b = Net("SW"), Net("FB"), Net("EN"), Net("BOOT"), Net("BUCK_VCC")
u_buck["VIN"] += vin
u_buck["PGND", "EP"] += gnd
u_buck["EN"] += en
u_buck["SW"] += sw
u_buck["BOOT"] += boot
u_buck["FB"] += fb
u_buck["VCC"] += vcc_b
r_ent, r_enb = R("3M", "C23156"), R("750k", "C23240")  # on ~6.2 V, off ~5.6 V
vin & r_ent & en & r_enb & gnd
r_fbt, r_fbb = R("1M", R_LCSC["1M"]), R("240k", "C4197")  # Vout = 1.0 * (1 + 1M/240k) = 5.17 V
v5r & r_fbt & fb & r_fbb & gnd
c_ff = C("10p C0G", "C1634")
c_ff[1] += v5r
c_ff[2] += fb
c_vcc = C("1u", C1U)
c_vcc[1] += vcc_b
c_vcc[2] += gnd
c_boot = C("100n", C100N)
c_boot[1] += boot
c_boot[2] += sw
l_buck = ic("MWSA0605S-150MT", "15u", "IND-SMD_L7.3-W6.8", "C408468", "L")
l_buck[1] += sw
l_buck[2] += v5r
for _ in range(2):
    c = C("22u 25V", "C12891", "Capacitor_SMD:C_1206_3216Metric")
    c[1] += v5r
    c[2] += gnd
c_out_hf = C("100n", C100N)
c_out_hf[1] += v5r
c_out_hf[2] += gnd

# OR-ing diode into the Heltec 5V pin. Header 5V is the Heltec's USB VBUS net; the
# diode drop keeps H_5V (~4.8 V) under USB's 5.0 V so a connected laptop wins and
# is never back-fed.
d_or = ic("SS34_C8678", "SS34", "SMA_L4.3-W2.6-LS5.2-RD", "C8678", "D")
d_or["A"] += v5r
d_or["K"] += h5v

# ---------------------------------------------------------------- always-on 3.3 V + reference
# Runs from VIN so the comparators keep working with the buck off. HT7533 is rated
# 30 V: 10k + 24 V zener hold its input below that through a load dump (the TVS
# clamps VIN at ~42 V); the 10k drops only ~0.2 V at the ~20 uA it carries.
v3a_in = Net("V3A_IN")
vin & R("10k", R_LCSC["10k"]) & v3a_in
d_z = ic("BZT52C24", "BZT52C24", "SOD-123_L2.7-W1.6-LS3.7-RD", "C173422", "D")
d_z["K"] += v3a_in
d_z["A"] += gnd
u_ldo = ic("HT7533-1_C14289", "HT7533-1", "SOT-89-3_L4.5-W2.5-P1.50-LS4.2-BR", "C14289")
u_ldo["Vin"] += v3a_in
u_ldo["Vout"] += v3a
u_ldo["GND"] += gnd
u_vref = ic("REF3325AIDBZR", "REF3325 2.5V", "SOT-23_L2.9-W1.3-P1.90-LS2.4-BR", "C69039")
u_vref["IN"] += v3a
u_vref["OUT"] += vref
u_vref["GND"] += gnd
for n, val, code in ((v3a_in, "1u", C1U), (v3a, "10u", "C15850"), (vref, "100n", C100N)):
    c = C(val, code, "Capacitor_SMD:C_0805_2012Metric" if val == "10u" else C0603)
    c[1] += n
    c[2] += gnd

# ---------------------------------------------------------------- low-voltage cutoff
# Holds the buck off below ~11.6 V (car battery getting low, still starts) and lets
# it run again above ~12.0 V: worst case off 11.39..11.83 V, on 11.77..12.23 V
# (sim/uvlo_thr.py). Measured key-on sag is 12.0 V (RAV4); resting 12.5 V on both cars. The 6 s filter ignores parked load spikes and rides through
# short cranking dips. Mainly a guard against firmware that fails to sleep.
uv_tap, uv_ref, uv_low = Net("UV_TAP"), Net("UV_REF"), Net("UV_LOW")
u_cmp = ic("TLV7022DDFR", "TLV7022DDFR", "SOT-23-8_L2.9-W1.6-P0.65-LS2.8-BR", "C2862588")
u_cmp["VCC"] += v3a
u_cmp["VEE"] += gnd
c = C("100n", C100N)
c[1] += v3a
c[2] += gnd
vin & R("3M", "C23156") & uv_tap & R("750k", "C23240") & gnd
c = C("10u", "C15850", "Capacitor_SMD:C_0805_2012Metric")
c[1] += uv_tap
c[2] += gnd
vref & R("510k", "C23192") & uv_ref & R("10M", "C7250") & gnd
u_cmp["INB-"] += uv_tap
u_cmp["INB+"] += uv_ref
u_cmp["OUTB"] += uv_low  # open drain: released (high) = battery low
uv_hys = Net("UV_HYS")
uv_low & R("10M", "C7250") & uv_hys & R("10M", "C7250") & uv_ref  # hysteresis, 20M (not stocked in 0603)
v3a & R("1M", R_LCSC["1M"]) & uv_low
q_uv = ic("2N7002", "2N7002", "SOT-23-3_L2.9-W1.3-P1.90-LS2.4-BR", "C8545", "Q")
q_uv["D"] += en
q_uv["G"] += uv_low
q_uv["S"] += gnd

# ---------------------------------------------------------------- Heltec V4.3 sockets
# H_J2 = Heltec JP4 (GND, 5V, Ve, Ve, 44, 43, RST, 0, 36, 35, 34, 33, 47, 48, 26, 21, 20, 19)
# H_J3 = Heltec JP5 (GND, 3V3, 3V3, 37, 46, 45, 42, 41, 40, 39, 38, 1, 2, 3, 4, 5, 6, 7)
h_j2 = ic("KH-2.54FH-1X18P-H8.5", "Heltec JP4", "HDR-TH_18P-P2.54-V-F", "C2905422", ref="J4")
h_j3 = ic("KH-2.54FH-1X18P-H8.5", "Heltec JP5", "HDR-TH_18P-P2.54-V-F", "C2905422", ref="J5")
gnss_en_n = Net("GNSS_EN_N")  # GPIO34 VGNSS_Ctrl, 10k pull-up on the Heltec
h_j2[1] += gnd
h_j2[2] += h5v
h_j2[3, 4] += vext
h_j2[11] += gnss_en_n
can_tx, can_rx = Net("CAN_TX"), Net("CAN_RX")
gps_pps, gps_txd, gps_rxd = Net("GPS_PPS"), Net("GPS_TXD"), Net("GPS_RXD")
h_j3[1] += gnd
h_j3[2, 3] += v3v3
h_j3[8] += gps_pps  # GPIO41
h_j3[10] += gps_txd  # GPIO39 = MCU RX (MeshCore Serial1.setPins(PIN_GPS_TX, PIN_GPS_RX))
h_j3[11] += gps_rxd  # GPIO38 = MCU TX
h_j3[14] += can_tx  # GPIO3
h_j3[15] += can_rx  # GPIO4

NO_CONNECT_J2 = [5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18]
NO_CONNECT_J3 = [4, 5, 6, 7, 9, 12, 13, 16, 17, 18]

# ---------------------------------------------------------------- CAN
# Powered from VEXT so it draws nothing while the Heltec sleeps; firmware must
# drive GPIO36 low before starting TWAI and keep GPIO3 low/hi-Z while VEXT is off.
u_can = ic("SN65HVD230DR", "SN65HVD230DR", "SOIC-8_L4.9-W3.9-P1.27-LS6.0-BL", "C12084")
u_can["VCC"] += vext
u_can["GND"] += gnd
u_can["D"] += can_tx
u_can["R"] += can_rx
u_can["RS"] += gnd  # high-speed mode
u_can["CANH"] += canh
u_can["CANL"] += canl
c = C("100n", C100N)
c[1] += vext
c[2] += gnd
esd = ic("PESD1CAN,215", "PESD1CAN", "SOT-23_L2.9-W1.3-P1.90-LS2.4-BR", "C15771", "D")
esd[1] += canl
esd[2] += canh
esd[3] += gnd
# Termination: jumper ON for the bench, OFF in the car
term = Net("CAN_TERM")
canh & R("120", R_LCSC["120"]) & term
jp_term = ic("254PZ-1X2P-V", "TERM 120R", "HDR-TH_2P-P2.54-V-M_2", "C53055672", ref="JP1")
jp_term[1] += term
jp_term[2] += canl

# ---------------------------------------------------------------- GNSS (GT-U7 on a 1x5 socket)
gnss_vcc, gnss_g = Net("GNSS_VCC"), Net("GNSS_G")
q_gnss = ic("AO3401A", "AO3401A", "SOT-23_L2.9-W1.3-P1.90-LS2.4-BR", "C15127", "Q")
q_gnss["S"] += v3v3
q_gnss["D"] += gnss_vcc
q_gnss["G"] += gnss_g
gnss_en_n & R("10k", R_LCSC["10k"]) & gnss_g  # with C_GS: ~100 us soft start, no 3V3 dip
c = C("10n", "C57112")
c[1] += gnss_g
c[2] += v3v3
c = C("1u", C1U)
c[1] += gnss_vcc
c[2] += gnd
# Footprint from Qeteshpony/GT-U7_GPS-Module (MIT): module outline, its mounting hole,
# and the pin numbering of that library (1 VCC, 2 GND, 3 RXD, 4 TXD, 5 PPS).
j_gnss = part(RN, "Header-Female-2.54_1x5", "GT-U7", "rn_carrier:GT-U7_THT", "C50950", ref="J2")
j_gnss[1] += gnss_vcc
j_gnss[2] += gnd
j_gnss[3] += gps_rxd
j_gnss[4] += gps_txd
j_gnss[5] += gps_pps

# ---------------------------------------------------------------- test points and holes
for name, n in [("VIN", vin), ("V5R", v5r), ("H5V", h5v), ("V3A", v3a), ("GND", gnd)]:
    tp = part("Connector", "TestPoint", name, "TestPoint:TestPoint_Pad_D1.5mm", "", "TP")
    tp[1] += n
for _ in range(4):
    part("Mechanical", "MountingHole", "M3", "MountingHole:MountingHole_3.2mm_M3", "", "MH")

# ---------------------------------------------------------------- engine-running wake (PROJ-595)
# VIN > ~13.3 V (charging: running, or Ready on the hybrid) releases CAR_ON
# high; it drops again below ~13.0 V. Key-on alone (12-12.5 V) stays low.
# GPIO21 is the only free RTC pin on the headers (OLED reset; RoadNode builds
# without the OLED). Firmware: ext0 wake on high, no internal pull. There is no
# free ADC pin, so the threshold is set here, not in firmware.
# Worst case on 13.03..13.54 V, off 12.71..13.22 V (sim/wake_thr.py).
# Second half of the low-voltage cutoff's TLV7022.
car_on, v_tap, v_ref = Net("CAR_ON"), Net("V_TAP"), Net("WAKE_REF")
vin & R("1M", R_LCSC["1M"]) & v_tap & R("220k", "C22961") & gnd  # 14.8 V -> 2.67 V
c = C("100n", C100N)  # ~20 ms, ignores ripple and spikes
c[1] += v_tap
c[2] += gnd
vref & R("47k", "C25819") & v_ref & R("750k", "C23240") & gnd  # 2.35 V
u_cmp["INA+"] += v_tap
u_cmp["INA-"] += v_ref
u_cmp["OUTA"] += car_on
car_on & R("10M", "C7250") & v_tap  # hysteresis
# Load dump: with VIN at the 42 V TVS clamp the tap would reach ~7.6 V, over the TLV7022's input
# rating. Silicon (not Schottky: nA leakage, so the threshold doesn't move hot) diode to V3A
# holds it near 3.9 V; the ~38 uA it feeds into V3A for a 400 ms dump lifts the rail <1.5 V.
# UV_TAP needs none: its 6 s filter moves <0.4 V in a 400 ms dump.
d_clamp = ic("1N4148WST4", "1N4148WS", "SOD-323_L1.8-W1.3-LS2.5-RD", "C2128", "D")
d_clamp["A"] += v_tap
d_clamp["K"] += v3a
v3a & R("1M", R_LCSC["1M"]) & car_on  # open drain, never above 3.3 V
h_j2[16] += car_on  # GPIO21
tp = part("Connector", "TestPoint", "WAKE", "TestPoint:TestPoint_Pad_D1.5mm", "", "TP")
tp[1] += car_on


if __name__ == "__main__":
    for p in NO_CONNECT_J2:
        h_j2[p] += NC
    for p in NO_CONNECT_J3:
        h_j3[p] += NC
    u_buck["PG"] += NC
    u_can["VREF"] += NC
    ERC()
    # Sequential refs per prefix in creation order (explicit refs kept), then
    # tag = ref so the PCB re-links to the same footprints on every run.
    parts = builtins.default_circuit.parts
    taken = {p.ref for p in parts if p.fixed_ref}
    for p in parts:
        if not p.fixed_ref:
            p.ref = "~" + p.ref
    for p in parts:
        if not p.fixed_ref:
            n = 1
            while f"{p.ref_prefix}{n}" in taken:
                n += 1
            p.ref = f"{p.ref_prefix}{n}"
            taken.add(p.ref)
    for p in parts:
        p.tag = p.ref
    generate_netlist(file_=os.path.join(HERE, "carrier.net"))
