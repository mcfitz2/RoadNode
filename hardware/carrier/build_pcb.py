"""Builds carrier.kicad_pcb from carrier.net: footprints, nets, placement, outline.

Run after circuit.py, with the venv (KiCad's Python, pcbnew importable):

    .venv/bin/python build_pcb.py

Rebuilds the board from scratch each time (placement lives in PLACE below), so
routing done by hand in KiCad is lost on rebuild: once routing starts, edit the
board in KiCad and use this script only for the first pass.

Coordinates are mm from the board's top-left corner, KiCad convention (y down).
"""
import os
import re
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
KICAD_FP = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
LIBS = {"rn_parts": os.path.join(HERE, "lib/rn_parts.pretty"), "rn_carrier": os.path.join(HERE, "lib/rn_carrier.pretty")}

# One-sided: the Heltec (USB-C at the left edge) and the GT-U7 sit end to end on top, ~8.5 mm up
# on their sockets, one header row along the top edge. Low SMD parts go under the modules (the
# comparators, references and dividers under the Heltec between its header rows); tall parts
# (C1, L1), the input stage and the test points go in the open strip below them. Everything
# assembled is on top, so JLC assembly stays one-sided.
BOARD_W, BOARD_H = 92.0, 37.0
ROW_TOP = 1.6  # top-edge pin row (Heltec J3 side, GT-U7)
HELTEC = (5.5, ROW_TOP + 22.86)  # Heltec J2-1 (GND, USB end)
HOLES = [(3.2, 14.0), (88.8, 14.0), (3.2, 33.8), (88.8, 33.8)]
# 2.4 GHz / LoRa end of the Heltec: no copper on the carrier underneath it.
ANTENNA_KEEPOUT = (46.5, 0.6, 61.5, 27.2)
# Hand-soldered: connectors, plus the Extended parts big enough for an iron (fuse, inductor,
# CAN ESD). JLC charges a placement fee plus an Extended-part fee for each. They keep their
# LCSC codes and are ordered loose from LCSC (tools/export_fab.py writes fab/lcsc_handfit.csv).
HAND_FIT = {"J1", "J2", "J4", "J5", "JP1", "F1", "L1", "D4"}

# ref: (x, y, rotation_deg, bottom_side)
PLACE = {
    # Heltec sockets: footprint origin is the row centre, pin 1 at -21.59
    "J4": (HELTEC[0] + 21.59, HELTEC[1], 0, False),
    "J5": (HELTEC[0] + 21.59, HELTEC[1] - 22.86, 0, False),
    # strip below the modules (y 28-36.4), left to right: vehicle input, bulk cap, buck,
    # test points, termination jumper; tall parts (J1, C1, L1, JP1) stay out from under the modules
    "J1": (13.5, 32.3, 0, False),
    "F1": (22.5, 30.0, 0, False),
    "D1": (22.5, 34.1, 0, False),
    "TP5": (27.1, 29.9, 0, False),
    "TP1": (27.1, 33.6, 0, False),
    "C1": (33.8, 32.2, 0, False),
    "C2": (41.4, 29.7, 180, False),
    "C3": (41.4, 32.4, 180, False),
    "Q1": (41.4, 35.0, 90, False),  # low-voltage cutoff: pulls buck EN low
    "U1": (47.8, 30.6, 270, False),
    "C6": (53.0, 28.8, 90, False),
    "R1": (44.0, 34.9, 90, False),  # EN divider
    "R2": (45.75, 34.9, 90, False),
    "C5": (53.0, 31.85, 90, False),
    "C4": (49.0, 34.9, 90, False),  # FB feed-forward cap + divider
    "R3": (50.75, 34.9, 90, False),
    "R4": (52.5, 34.9, 90, False),
    "L1": (58.5, 32.2, 90, False),
    "C7": (63.8, 31.4, 90, False),
    "C8": (66.6, 31.4, 90, False),
    "C9": (69.2, 31.4, 90, False),
    "TP2": (72.2, 30.2, 0, False),
    "JP1": (79.5, 30.2, 0, False),
    "R13": (79.5, 33.5, 0, False),
    # under the Heltec (low parts only): CAN by J5 pins 14/15, OR diode by the 5V pin
    "U5": (40.0, 8.5, 90, False),
    "C15": (44.5, 9.0, 90, False),
    "D4": (22.0, 20.8, 90, False),  # CAN ESD
    "D2": (12.0, 20.8, 0, False),
    "TP3": (7.5, 20.8, 0, False),
    "TP4": (18.5, 20.8, 0, False),
    # always-on 3.3 V (zener-protected) and the 2.5 V reference
    "D3": (10.0, 4.5, 0, False),
    "R5": (10.0, 7.5, 0, False),
    "C10": (14.5, 7.5, 0, False),
    "U2": (11.0, 11.5, 0, False),
    "C11": (15.0, 11.5, 90, False),
    "U3": (33.0, 13.0, 0, False),
    "C12": (36.5, 13.0, 90, False),
    # dual comparator: engine-running wake (A) and low-voltage cutoff (B)
    "U4": (24.0, 8.5, 0, False),
    "C13": (20.0, 8.5, 90, False),
    "R15": (18.0, 5.0, 0, False),  # wake VIN divider
    "R16": (21.5, 5.0, 0, False),
    "C18": (25.0, 5.0, 0, False),
    "D5": (35.5, 5.0, 0, False),  # wake tap load-dump clamp to V3A
    "R19": (28.5, 5.0, 0, False),  # wake hysteresis
    "R17": (28.5, 8.5, 0, False),  # wake reference divider
    "R18": (32.0, 8.5, 0, False),
    "R20": (24.0, 12.5, 0, False),  # CAR_ON pull-up
    "TP6": (29.0, 13.0, 0, False),
    "R6": (11.0, 16.5, 0, False),  # cutoff VIN divider + 6 s filter
    "R7": (14.5, 16.5, 0, False),
    "C14": (18.0, 16.5, 0, False),
    "R8": (21.5, 16.5, 0, False),  # cutoff reference, hysteresis, pull-up
    "R9": (25.0, 16.5, 0, False),
    "R10": (28.5, 16.5, 0, False),
    "R11": (32.0, 16.5, 0, False),
    "R12": (35.5, 16.5, 0, False),
    # GT-U7 on its socket: origin is the module's top edge centre, pins 1.27 below it,
    # body over x 62-90 / y 0.3-27.6 (U.FL bottom right); its switch sits under it
    "J2": (76.0, ROW_TOP - 1.27, 0, False),
    "Q2": (72.0, 6.5, 0, False),
    "R14": (68.5, 5.5, 0, False),
    "C16": (68.5, 8.0, 0, False),
    "C17": (77.5, 6.5, 90, False),
}


# Net classes, written into carrier.kicad_pro (the autorouter reads widths from there).
# 0.6 mm: 1 oz copper carries ~1.5 A at 10 C rise, and still fits the HSOIC pads.
NET_CLASSES = {
    "Power": (0.6, 0.25, ["V_CAR", "VIN", "SW", "V5R", "H_5V"]),
    # GND is a plane to the autorouter (both layers poured); this width is for its fanout stubs
    "Gnd": (0.4, 0.2, ["GND"]),
    "Rail": (0.4, 0.2, ["3V3", "GNSS_VCC"]),  # VEXT feeds only the CAN transceiver: default width
}


def set_net_classes(pro_path):
    import json
    pro = json.load(open(pro_path))
    ns = pro["net_settings"]
    default = next(c for c in ns["classes"] if c["name"] == "Default")
    default["track_width"] = 0.25
    ns["classes"] = [default]
    ns["netclass_patterns"] = []
    for prio, (name, (width, clearance, nets)) in enumerate(NET_CLASSES.items()):
        ns["classes"].append(dict(default, name=name, track_width=width, clearance=clearance, priority=prio))
        ns["netclass_patterns"] += [{"netclass": name, "pattern": n} for n in nets]
    # JLC 2-layer limits are 0.127 mm track / 0.3 mm to edge; Freerouting necks down to ~0.19 mm at
    # fine-pitch pads and runs some tracks 0.4 mm from the edge.
    rules = pro["board"]["design_settings"]["rules"]
    rules.update(min_track_width=0.15, min_copper_edge_clearance=0.3)
    json.dump(pro, open(pro_path, "w"), indent=2)


def parse_netlist(path):
    """Returns ({ref: (footprint, value, {field: value})}, {net: [(ref, pin)]})."""
    text = open(path).read()
    comps_text, nets_text = text.split("(nets", 1)
    comps = {}
    for comp in re.split(r"\(comp\s", comps_text)[1:]:
        ref = re.search(r'\(ref "([^"]+)"\)', comp).group(1)
        value = re.search(r'\(value "([^"]*)"\)', comp).group(1)
        fp = re.search(r'\(footprint "([^"]*)"\)', comp).group(1)
        fields = dict(re.findall(r'\(name "([^"]+)"\) "([^"]*)"\)', comp))
        comps[ref] = (fp, value, fields)
    nets = {}
    for net in re.split(r"\(net\s", nets_text)[1:]:
        name = re.search(r'\(name "([^"]+)"\)', net).group(1)
        nets[name] = re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"\)', net)
    return comps, nets


def load_fp(lib_id):
    lib, name = lib_id.split(":")
    path = LIBS.get(lib, os.path.join(KICAD_FP, lib + ".pretty"))
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        sys.exit(f"footprint not found: {lib_id}")
    return fp


def mm(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def add_outline(board):
    pts = [(0, 0), (BOARD_W, 0), (BOARD_W, BOARD_H), (0, BOARD_H)]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        seg = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(mm(*a))
        seg.SetEnd(mm(*b))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(pcbnew.FromMM(0.1))
        board.Add(seg)


def rect_outline(x0, y0, x1, y1):
    poly = pcbnew.SHAPE_LINE_CHAIN()
    for x, y in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]:
        poly.Append(mm(x, y))
    poly.SetClosed(True)
    return poly


def add_zone(board, net, layer, keepout=False, rect=(0.3, 0.3, BOARD_W - 0.3, BOARD_H - 0.3), footprints=False):
    zone = pcbnew.ZONE(board)
    zone.Outline().AddOutline(rect_outline(*rect))
    if keepout:
        # No pours on either layer; tracks only on B.Cu (layer=B_Cu passes them, F_Cu blocks them).
        # The Heltec sits ~8.5 mm up on its sockets, so a few thin bottom tracks are tolerable;
        # a ground plane under the antenna is not.
        zone.SetIsRuleArea(True)
        zone.SetDoNotAllowTracks(layer == pcbnew.F_Cu or keepout != True)
        zone.SetDoNotAllowVias(True)
        zone.SetDoNotAllowZoneFills(True)
        zone.SetDoNotAllowPads(False)
        zone.SetDoNotAllowFootprints(footprints)
        zone.SetLayer(layer)
        zone.SetZoneName("antenna keepout" if keepout is True else keepout)
    else:
        zone.SetLayer(layer)
        zone.SetNet(net)
        zone.SetMinThickness(pcbnew.FromMM(0.25))
        zone.SetLocalClearance(pcbnew.FromMM(0.2))
        zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)  # SMD pads solid, THT relieved
        zone.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    board.Add(zone)


LOGO = os.path.join(HERE, "lib/roadnode_logo.svg")
# Bottom silkscreen: the top is all parts, the bottom carries none. Placed in board coordinates
# (as seen from the top) and mirrored so it reads right from below.
LOGO_BOX = (39.0, 5.0, 55.0, 21.5)  # between the Heltec rows, clear of the header pins
CREDIT_AT = (47.0, 28.6)  # centre of the two text lines below the logo
CREDIT = ["Micah Fitzgerald", "github.com/mcfitz2/RoadNode"]


def add_logo(board, path, box, layer=pcbnew.B_SilkS):
    """Logo SVG onto a silk layer as filled polygons, fitted into box (x0, y0, x1, y1).

    Handles the one form the logo uses: black paths of straight segments
    (M/L/Z), even-odd fill. Anything else stops the build rather than drawing wrong art.
    """
    svg = open(path).read()
    rings = []
    for d in re.findall(r'<path[^>]*\sd="([^"]+)"', svg):
        if re.search(r"[^MLZ\d\s.,-]", d):
            sys.exit(f"{path}: only M/L/Z path commands are supported")
        for sub in re.split(r"Z", d):
            nums = [float(n) for n in re.findall(r"-?\d+(?:\.\d+)?", sub)]
            if len(nums) >= 6:
                rings.append(list(zip(nums[0::2], nums[1::2])))
    xs = [x for r in rings for x, _ in r]
    ys = [y for r in rings for _, y in r]
    w, h = max(xs) - min(xs), max(ys) - min(ys)
    s = min((box[2] - box[0]) / w, (box[3] - box[1]) / h)
    if layer == pcbnew.B_SilkS:  # mirror about the box centre
        cx = (min(xs) + max(xs)) / 2
        rings = [[(2 * cx - x, y) for x, y in r] for r in rings]
    ox = (box[0] + box[2]) / 2 - s * (min(xs) + max(xs)) / 2
    oy = (box[1] + box[3]) / 2 - s * (min(ys) + max(ys)) / 2

    art = pcbnew.SHAPE_POLY_SET()
    for r in rings:  # XOR of every ring is exactly the even-odd fill
        ring = pcbnew.SHAPE_POLY_SET()
        chain = pcbnew.SHAPE_LINE_CHAIN()
        for x, y in r:
            chain.Append(mm(ox + s * x, oy + s * y))
        chain.SetClosed(True)
        ring.AddOutline(chain)
        art.BooleanXor(ring)
    art.Fracture()  # holes bridged into their outline: one simple polygon per island
    for i in range(art.OutlineCount()):  # the file keeps only the first outline of a shape
        island = pcbnew.SHAPE_POLY_SET()
        island.AddOutline(art.Outline(i))
        shape = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_POLY)
        shape.SetPolyShape(island)
        shape.SetFilled(True)
        shape.SetWidth(0)
        shape.SetLayer(layer)
        board.Add(shape)


def add_text(board, lines, at, size=1.0, thickness=0.15, layer=pcbnew.B_SilkS):
    for i, line in enumerate(lines):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(line)
        t.SetLayer(layer)
        t.SetMirrored(layer == pcbnew.B_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
        t.SetTextThickness(pcbnew.FromMM(thickness))
        t.SetPosition(mm(at[0], at[1] + (i - (len(lines) - 1) / 2) * size * 1.6))
        board.Add(t)


def main():
    comps, nets = parse_netlist(os.path.join(HERE, "carrier.net"))
    board = pcbnew.BOARD()
    board.GetDesignSettings().SetCopperLayerCount(2)

    netinfo = {}
    for name in nets:
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        netinfo[name] = ni
    pin_net = {(ref, pin): name for name, nodes in nets.items() for ref, pin in nodes}

    staging_x = BOARD_W + 10
    unplaced = []
    for ref, (lib_id, value, fields) in sorted(comps.items()):
        fp = load_fp(lib_id)
        fp.SetReference(ref)
        fp.SetValue(value)
        for k, v in fields.items():
            if k in ("LCSC",):
                fp.SetField(k, v)
        if not fields.get("LCSC") or ref in HAND_FIT:
            fp.SetBoardOnly(False)
            fp.SetExcludedFromBOM(True)
            fp.SetExcludedFromPosFiles(True)
        for f in fp.GetFields():
            if not f.IsReference():
                f.SetVisible(False)
        board.Add(fp)
        for pad in fp.Pads():
            name = pin_net.get((ref, pad.GetNumber()))
            if name:
                pad.SetNet(netinfo[name])
        if ref in PLACE:
            x, y, rot, bottom = PLACE[ref]
            fp.SetPosition(mm(x, y))
            fp.SetOrientationDegrees(rot)
            if bottom:
                fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM)
                # hand-fit, so JLC assembles one side only
                fp.SetExcludedFromBOM(True)
                fp.SetExcludedFromPosFiles(True)
        elif ref.startswith("MH") and int(ref[2:]) <= len(HOLES):
            fp.SetPosition(mm(*HOLES[int(ref[2:]) - 1]))
        else:
            unplaced.append(ref)
            fp.SetPosition(mm(staging_x, 5 + 6 * len(unplaced)))

    # Heltec outline + 3D model for the fit check; pads removed (the sockets carry them)
    heltec = load_fp("rn_carrier:Heltec_WiFi_LoRa32_V4")
    for item in list(heltec.Pads()) + [g for g in heltec.GraphicalItems() if g.GetLayer() == pcbnew.F_CrtYd]:
        heltec.Remove(item)
    heltec.SetReference("MOD1")
    heltec.SetValue("Heltec WiFi LoRa 32 V4.3 (on sockets)")
    heltec.SetBoardOnly(True)
    heltec.SetExcludedFromBOM(True)
    heltec.SetExcludedFromPosFiles(True)
    heltec.SetPosition(mm(*HELTEC))
    board.Add(heltec)

    add_outline(board)
    add_logo(board, LOGO, LOGO_BOX)
    add_text(board, CREDIT, CREDIT_AT)
    gnd = netinfo["GND"]
    add_zone(board, gnd, pcbnew.F_Cu)
    add_zone(board, gnd, pcbnew.B_Cu)
    add_zone(board, None, pcbnew.F_Cu, keepout=True, rect=ANTENNA_KEEPOUT)
    add_zone(board, None, pcbnew.B_Cu, keepout=True, rect=ANTENNA_KEEPOUT)
    # 0.6 mm edge band: the autorouter only knows the outline, JLC wants 0.3 mm copper-to-edge
    e = 0.6
    for i, rect in enumerate([(0, 0, BOARD_W, e), (0, BOARD_H - e, BOARD_W, BOARD_H),
                              (0, 0, e, BOARD_H), (BOARD_W - e, 0, BOARD_W, BOARD_H)]):
        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            add_zone(board, None, layer, keepout=f"edge band {i}", rect=rect)

    out = os.path.join(HERE, "carrier.kicad_pcb")
    board.Save(out)
    set_net_classes(os.path.join(HERE, "carrier.kicad_pro"))
    print(f"wrote {out}: {len(comps)} parts, {len(nets)} nets")
    if unplaced:
        print("UNPLACED (staged right of the board):", " ".join(unplaced))


if __name__ == "__main__":
    main()
