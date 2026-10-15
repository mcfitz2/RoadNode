"""RoadNode OBD carrier board: circuit description.

Generates carrier.net with SKiDL (SKiDL's schematic generator cannot place and
route a circuit this size, so the netlist is the source of truth). Run with the venv in
this directory (KiCad's bundled Python + skidl):

    .venv/bin/python circuit.py

Design notes and the reasoning behind each value live in notes/design.md; the
NTC gate values come from sim/ntc_calc.py and are checked by sim/ntc_gate*.cir
and sim/ntc_mc.py.
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
v5r = Net("V5R")  # buck output, always on while plugged in
v5g = Net("V5G")  # after the temperature-gated load switch
h5v = Net("H_5V")  # Heltec header 5V pin (= its USB VBUS net)
v3ref = Net("V3REF")  # 3.3 V reference for the NTC divider and thresholds
v3v3 = Net("3V3")  # Heltec always-on 3.3 V (header)
vext = Net("VEXT")  # Heltec switched 3.3 V (GPIO36 low = on)
canh, canl = Net("CANH"), Net("CANL")
bat_p, bat_n = Net("BAT_P"), Net("BAT_N")
for n in (vin, v5r, v3v3, vext, bat_p):
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

# ---------------------------------------------------------------- NTC charge gate (sim/)
# Charging is allowed only while the cell NTC reads ~1..44 C. Outside that
# window the load switch opens, the Heltec loses 5 V and runs from the cells.
u_ref = ic("XC6206P302MR", "XC6206P332MR 3.3V", "SOT-23-3_L2.9-W1.6-P1.90-LS2.8-BR", "C5446")  # same pinout as the 3.0 V symbol
u_ref["Vin"] += v5r
u_ref["Vout"] += v3ref
u_ref["GND"] += gnd
for n in (v5r, v3ref):
    c = C("1u", C1U)
    c[1] += n
    c[2] += gnd

ntc_t, ha, hb = Net("NTC_T"), Net("HA"), Net("HB")
temp_ok, too_hot, chg_gate = Net("TEMP_OK"), Net("TOO_HOT"), Net("CHG_GATE")
r_ntc_top = R("100k", R_LCSC["100k"])
v3ref & r_ntc_top & ntc_t
th = ic("SDNT1608X104F3950FTF", "100k B3950", "R0603", "C919169", ref="TH1")  # place between the cells
th[1] += ntc_t
th[2] += gnd
c_ntc = C("100n", C100N)
c_ntc[1] += ntc_t
c_ntc[2] += gnd
r_adc = R("1k", R_LCSC["1k"])  # NTC_T -> GPIO6 (ADC1_CH5), never above 3.3 V
ntc_adc = Net("NTC_ADC")
ntc_t & r_adc & ntc_adc

u_cmp = ic("TLV7022DDFR", "TLV7022DDFR", "SOT-23-8_L2.9-W1.6-P0.65-LS2.8-BR", "C2862588")
u_cmp["VCC"] += v5r
u_cmp["VEE"] += gnd
c = C("100n", C100N)
c[1] += v5r
c[2] += gnd
# A (cold): out high = warm enough
u_cmp["INA+"] += ha
u_cmp["INA-"] += ntc_t
u_cmp["OUTA"] += temp_ok
v3ref & R("100k", R_LCSC["100k"]) & ha & R("300k", "C23024") & gnd
temp_ok & R("5.1M", "C13320") & ha
v5r & R("1M", R_LCSC["1M"]) & temp_ok
# B (hot): out high = too hot, which pulls TEMP_OK low through Q_HOT
u_cmp["INB+"] += hb
u_cmp["INB-"] += ntc_t
u_cmp["OUTB"] += too_hot
v3ref & R("470k", "C23178") & hb & R("220k", "C22961") & gnd
too_hot & R("10M", "C7250") & hb
v5r & R("1M", R_LCSC["1M"]) & too_hot


def nfet():
    return ic("2N7002", "2N7002", "SOT-23-3_L2.9-W1.3-P1.90-LS2.4-BR", "C8545", "Q")


q_hot = nfet()
q_hot["D"] += temp_ok
q_hot["G"] += too_hot
q_hot["S"] += gnd
q_drv = nfet()
q_drv["D"] += chg_gate
q_drv["G"] += temp_ok
q_drv["S"] += gnd

# Load switch + OR-ing diode into the Heltec 5V pin. Header 5V is the Heltec's
# USB VBUS net; the diode drop keeps H_5V (~4.8 V) under USB's 5.0 V so a
# connected laptop wins and is never back-fed.
q_ls = ic("AO3401A", "AO3401A", "SOT-23_L2.9-W1.3-P1.90-LS2.4-BR", "C15127", "Q")
q_ls["S"] += v5r
q_ls["D"] += v5g
q_ls["G"] += chg_gate
v5r & R("1M", R_LCSC["1M"]) & chg_gate
d_or = ic("SS34_C8678", "SS34", "SMA_L4.3-W2.6-LS5.2-RD", "C8678", "D")
d_or["A"] += v5g
d_or["K"] += h5v

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
h_j3[17] += ntc_adc  # GPIO6

NO_CONNECT_J2 = [5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18]
NO_CONNECT_J3 = [4, 5, 6, 7, 9, 12, 13, 16, 18]

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

# ---------------------------------------------------------------- cells + protection
# Two unprotected 18650s in parallel, each through its own PTC. DW01A + 8205A dual FET
# low-side protection between the cell negative (BAT_N) and system ground.
for i in (1, 2):
    bt = part("Device", "Battery_Cell", "18650", "rn_carrier:BH_18650_TH_P72.4", "", ref=f"BT{i}")
    cell_p = Net(f"CELL{i}_P")
    bt[1] += cell_p
    bt[2] += bat_n
    f = ic("BSMD1206-200-6V", "2A hold", "F1206", "C883134", "F")
    f[1] += cell_p
    f[2] += bat_p
u_prot = ic("DW01A_C351410", "DW01A", "SOT-23-6_L2.9-W1.6-P0.95-LS2.8-BL", "C351410")
q_prot = ic("FS8205A_C2830320", "HJ8205", "SOT-23-6_L2.9-W1.6-P0.95-LS2.8-BR", "C20069150", "Q")  # FS8205A pinout
prot_vcc, od, oc, fet_d = Net("PROT_VCC"), Net("PROT_OD"), Net("PROT_OC"), Net("PROT_D")
bat_p & R("100", R_LCSC["100"]) & prot_vcc
c = C("100n", C100N)
c[1] += prot_vcc
c[2] += bat_n
u_prot["VCC"] += prot_vcc
u_prot["GND"] += bat_n
u_prot["OD"] += od
u_prot["OC"] += oc
vm = Net("PROT_VM")
u_prot["VM"] += vm
vm & R("1k", R_LCSC["1k"]) & gnd
q_prot["S1"] += bat_n
q_prot["G1"] += od
q_prot["S2"] += gnd
q_prot["G2"] += oc
q_prot["D1/D2"] += fet_d
# Pack out to the Heltec battery jack (JP2: 1 = VBAT+, 2 = GND) by a straight 1.25 mm pigtail
j_bat = ic("1.25T-2PWT", "to Heltec JP2", "CONN-SMD_P1.25-H_1.25T-2PWT", "C22074", ref="J3")
j_bat[1] += bat_p
j_bat[2] += gnd
j_bat[3, 4] += gnd  # mounting tabs

# ---------------------------------------------------------------- test points and holes
for name, n in [("VIN", vin), ("V5R", v5r), ("H5V", h5v), ("BATP", bat_p), ("NTC", ntc_t), ("GND", gnd)]:
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
# 1% resistors + XC6206 2%: about +-0.3 V on both thresholds (sim/wake_thr.py).
car_on, v_tap, v_ref = Net("CAR_ON"), Net("V_TAP"), Net("WAKE_REF")
u_wake = ic("TLV7022DDFR", "TLV7022DDFR", "SOT-23-8_L2.9-W1.6-P0.65-LS2.8-BR", "C2862588")
u_wake["VCC"] += v3ref
u_wake["VEE"] += gnd
c = C("100n", C100N)
c[1] += v3ref
c[2] += gnd
vin & R("1M", R_LCSC["1M"]) & v_tap & R("220k", "C22961") & gnd  # 14.8 V -> 2.67 V
c = C("100n", C100N)  # ~20 ms, ignores ripple and spikes
c[1] += v_tap
c[2] += gnd
v3ref & R("300k", "C23024") & v_ref & R("750k", "C23240") & gnd  # 2.36 V
u_wake["INA+"] += v_tap
u_wake["INA-"] += v_ref
u_wake["OUTA"] += car_on
car_on & R("10M", "C7250") & v_tap  # hysteresis
v3ref & R("1M", R_LCSC["1M"]) & car_on  # open drain, never above 3.3 V
u_wake["INB+"] += gnd  # unused half: output held low
u_wake["INB-"] += v3ref
h_j2[16] += car_on  # GPIO21
tp = part("Connector", "TestPoint", "WAKE", "TestPoint:TestPoint_Pad_D1.5mm", "", "TP")
tp[1] += car_on


if __name__ == "__main__":
    for p in NO_CONNECT_J2:
        h_j2[p] += NC
    for p in NO_CONNECT_J3:
        h_j3[p] += NC
    u_buck["PG"] += NC
    u_prot["TD"] += NC
    u_can["VREF"] += NC
    u_wake["OUTB"] += NC
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
