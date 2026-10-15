"""Writes JLCPCB fabrication + assembly files for carrier.kicad_pcb into fab/.

    .venv/bin/python tools/export_fab.py

fab/gerbers.zip   Gerbers + Excellon drill (upload as the PCB)
fab/bom.csv       Comment, Designator, Footprint, LCSC (JLC BOM format)
fab/cpl.csv       Designator, Mid X, Mid Y, Layer, Rotation (JLC CPL format)
fab/carrier.step  3D model for the enclosure / fit check
fab/lcsc_handfit.csv  hand-soldered parts to order loose from LCSC (BOM upload format)

Only parts with an LCSC code that are not marked hand-fit go into the BOM and CPL.
Rotations are KiCad's; JLC's viewer shows each part on its pad pattern before
the order is placed, so check polarity there (diodes, ICs, electrolytic).
"""
import csv
import os
import shutil
import subprocess
import tempfile
import zipfile

import pcbnew

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KICAD_CLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
LAYERS = "F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts"
BOARDS = 2  # boards to build from the loose parts; one spare per line on top (LCSC rounds up to its minimum)
EXTRA_LOOSE = [("C100114", "2.54 mm jumper shunt", "JP1 shunt", 1)]  # not on the board, ships with it


def cli(*args):
    subprocess.run([KICAD_CLI, *args], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    pcb = os.path.join(HERE, "carrier.kicad_pcb")
    out = os.path.join(HERE, "fab")
    os.makedirs(out, exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:
        cli("pcb", "export", "gerbers", "--layers", LAYERS, "--subtract-soldermask", "-o", tmp + "/", pcb)
        cli("pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th", "--generate-map",
            "--map-format", "gerberx2", "-o", tmp + "/", pcb)
        with zipfile.ZipFile(os.path.join(out, "gerbers.zip"), "w", zipfile.ZIP_DEFLATED) as z:
            for name in sorted(os.listdir(tmp)):
                z.write(os.path.join(tmp, name), name)

    board = pcbnew.LoadBoard(pcb)
    origin = board.GetDesignSettings().GetAuxOrigin()
    groups, cpl, loose = {}, [], {}
    for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
        lcsc = {f.GetName(): f.GetText() for f in fp.GetFields()}.get("LCSC", "")
        if not lcsc:
            continue
        if fp.IsExcludedFromBOM():
            loose.setdefault(lcsc, []).append(fp)
            continue
        key = (fp.GetValue(), fp.GetFPID().GetLibItemName().wx_str(), lcsc)
        groups.setdefault(key, []).append(fp.GetReference())
        pos = fp.GetPosition() - origin
        cpl.append([fp.GetReference(), f"{pcbnew.ToMM(pos.x):.3f}mm", f"{-pcbnew.ToMM(pos.y):.3f}mm",
                    "Bottom" if fp.IsFlipped() else "Top", f"{fp.GetOrientationDegrees() % 360:.0f}"])

    with open(os.path.join(out, "bom.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC"])
        for (value, footprint, lcsc), refs in sorted(groups.items(), key=lambda kv: kv[1][0]):
            w.writerow([value, ",".join(refs), footprint, lcsc])
    with open(os.path.join(out, "cpl.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        w.writerows(cpl)

    with open(os.path.join(out, "lcsc_handfit.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["LCSC Part Number", "Quantity", "Comment", "Designator"])
        rows = [(lcsc, fps[0].GetValue(), ",".join(f.GetReference() for f in fps), len(fps))
                for lcsc, fps in sorted(loose.items())]
        for lcsc, value, refs, per_board in rows + EXTRA_LOOSE:
            w.writerow([lcsc, per_board * BOARDS + 1, value, refs])

    cli("pcb", "export", "step", "--subst-models", "--force", "-o", os.path.join(out, "carrier.step"), pcb)
    print(f"wrote {out}: gerbers.zip, bom.csv ({len(groups)} lines, {len(cpl)} parts), cpl.csv, carrier.step, lcsc_handfit.csv ({len(loose) + len(EXTRA_LOOSE)} lines)")


if __name__ == "__main__":
    main()
