"""Autoroutes carrier.kicad_pcb with Freerouting and writes the result back.

Exports a Specctra DSN, runs Freerouting headless, imports the SES, refills the
zones and saves. Run with the venv (KiCad's Python):

    .venv/bin/python tools/autoroute.py [passes]

Needs Java 21+ and the Freerouting jar at .venv/freerouting.jar
(https://github.com/freerouting/freerouting/releases). Check the result with
kicad-cli pcb drc; the autorouter is a first pass, not a substitute for review.
"""
import os
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JAR = os.path.join(HERE, ".venv/freerouting.jar")
JAVA = "/opt/homebrew/opt/openjdk/bin/java"


def widen_necks(board, min_mm=0.15):
    """Freerouting necks tracks down to 0.125 mm at fine-pitch pads; JLC's 2-layer floor is 0.127 mm.
    Widens them to min_mm (DRC then confirms the clearance still holds)."""
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T and t.GetWidth() < pcbnew.FromMM(min_mm):
            t.SetWidth(pcbnew.FromMM(min_mm))


class Stitcher:
    """Adds GND vias so every piece of both GND pours is tied to the other layer.

    Freerouting treats the GND pours as planes, so it leaves them cut into pieces
    by tracks. A coarse grid of vias goes in first, then one targeted via in each
    pour fragment that still has none; vias that miss the pour on either layer are
    dropped again.
    """
    VIA_D, DRILL, GAP = pcbnew.FromMM(0.6), pcbnew.FromMM(0.3), pcbnew.FromMM(0.3)

    def __init__(self, board):
        self.board = board
        self.gnd = board.FindNet("GND")
        self.keepouts = [z for z in board.Zones() if z.GetIsRuleArea()]
        self.pours = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == "GND"]
        self.items = list(board.GetTracks()) + [p for fp in board.GetFootprints() for p in fp.Pads()]
        self.added = []
        self.gnd_pads = [p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetname() == "GND"]
        # drilled holes of any net (pads and vias): a via must keep hole-to-hole clearance from them too
        self.holes = [(p.GetPosition(), max(p.GetDrillSize().x, p.GetDrillSize().y) // 2)
                      for fp in board.GetFootprints() for p in fp.Pads() if p.GetDrillSize().x > 0]
        self.holes += [(t.GetPosition(), t.GetDrill() // 2) for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T]

    def clear(self, pos):
        if any(z.Outline().Contains(pos) for z in self.keepouts):
            return False
        if any((pos - c).EuclideanNorm() < r + self.DRILL // 2 + self.GAP for c, r in self.holes):
            return False
        return self.free(pcbnew.SHAPE_CIRCLE(pos, self.VIA_D // 2), (pcbnew.F_Cu, pcbnew.B_Cu))

    def free(self, shape, layers):
        """True if shape touches nothing but GND on the given layers."""
        for it in self.items:
            if it.GetNetname() == "GND":
                continue
            for layer in layers:
                if it.IsOnLayer(layer) and it.GetEffectiveShape(layer).Collide(shape, self.GAP):
                    return False
        return True

    def stub(self, pad, layer):
        """Short track from an SMD pad stranded on a pour sliver to an existing GND via,
        or else to a new via on a ring around the pad."""
        import math
        c = pad.GetPosition()
        width = pcbnew.FromMM(0.25)
        vias = sorted((t.GetPosition() for t in self.board.GetTracks()
                       if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == "GND"),
                      key=lambda v: (v - c).EuclideanNorm())
        targets = [(v, False) for v in vias if (v - c).EuclideanNorm() < pcbnew.FromMM(5)]
        for r_mm in (0.9, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0):
            for k in range(32):
                a = 2 * math.pi * k / 32
                targets.append((pcbnew.VECTOR2I(int(c.x + pcbnew.FromMM(r_mm) * math.cos(a)),
                                                int(c.y + pcbnew.FromMM(r_mm) * math.sin(a))), True))
        for pos, new_via in targets:
            if not self.free(pcbnew.SHAPE_SEGMENT(c, pos, width), (layer,)):
                continue
            if new_via:
                if not self.clear(pos):
                    continue
                self.add(pos)
            track = pcbnew.PCB_TRACK(self.board)
            track.SetStart(c)
            track.SetEnd(pos)
            track.SetWidth(width)
            track.SetLayer(layer)
            track.SetNet(self.gnd)
            self.board.Add(track)
            self.items.append(track)
            return True
        return False

    def add(self, pos):
        via = pcbnew.PCB_VIA(self.board)
        via.SetPosition(pos)
        via.SetWidth(self.VIA_D)
        via.SetDrill(self.DRILL)
        via.SetNet(self.gnd)
        self.board.Add(via)
        self.items.append(via)
        self.added.append(via)
        self.holes.append((pos, self.DRILL // 2))

    def fill(self):
        pcbnew.ZONE_FILLER(self.board).Fill(self.board.Zones())

    def in_both(self, pos):
        return len({z.GetLayer() for z in self.pours if z.GetFilledPolysList(z.GetLayer()).Contains(pos)}) == 2

    def groups(self):
        """Filled GND fragments of both pours and, for each, a connectivity group id: fragments
        are joined by GND vias, through-hole GND pads and GND tracks between them."""
        frags = [(z.GetLayer(), z.GetFilledPolysList(z.GetLayer()).Outline(i))
                 for z in self.pours for i in range(z.GetFilledPolysList(z.GetLayer()).OutlineCount())]
        parent = list(range(len(frags)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def join(ids):
            for a in ids[1:]:
                parent[find(a)] = find(ids[0])

        def touching(layer, shape):
            return [i for i, (l, f) in enumerate(frags) if l == layer and shape.Collide(f, 0)]

        for t in self.board.GetTracks():
            if t.GetNetname() != "GND":
                continue
            if t.Type() == pcbnew.PCB_VIA_T:
                join(touching(pcbnew.F_Cu, t.GetEffectiveShape(pcbnew.F_Cu))
                     + touching(pcbnew.B_Cu, t.GetEffectiveShape(pcbnew.B_Cu)))
            else:
                join(touching(t.GetLayer(), t.GetEffectiveShape(t.GetLayer())))
        for p in self.gnd_pads:
            ids = [i for layer in (pcbnew.F_Cu, pcbnew.B_Cu) if p.IsOnLayer(layer)
                   for i in touching(layer, p.GetEffectiveShape(layer))]
            if p.GetDrillSize().x > 0 or len({frags[i][0] for i in ids}) == 1:
                join(ids)  # SMD pads only join fragments on their own layer
        return frags, [find(i) for i in range(len(frags))]

    def grid(self, box, pitch, inside=None):
        """Yields grid points in box (and inside the polygon, if given)."""
        y = box.GetTop()
        while y <= box.GetBottom():
            x = box.GetLeft()
            while x <= box.GetRight():
                pos = pcbnew.VECTOR2I(int(x), int(y))
                if inside is None or inside.PointInside(pos):
                    yield pos
                x += pitch
            y += pitch

    def run(self, pitch_mm=5.0, margin_mm=1.5):
        box = self.board.GetBoardEdgesBoundingBox()
        box.Inflate(-pcbnew.FromMM(margin_mm))
        for pos in self.grid(box, pcbnew.FromMM(pitch_mm)):
            if self.clear(pos):
                self.add(pos)
        self.fill()
        for _ in range(4):
            frags, group = self.groups()
            main = max(set(group), key=group.count)
            stranded = [i for i in range(len(frags)) if group[i] != main]
            if not stranded:
                break
            done = set()
            for i in stranded:
                if group[i] in done:
                    continue
                layer, frag = frags[i]
                other = [j for j, (l, _) in enumerate(frags) if l != layer and group[j] != group[i]]
                # a via where this fragment overlaps a differently connected one on the other layer
                pos = next((p for p in self.grid(frag.BBox(), pcbnew.FromMM(0.25), inside=frag)
                            if any(frags[j][1].PointInside(p) for j in other) and self.clear(p)), None)
                if pos is not None:
                    self.add(pos)
                    done.add(group[i])
                    continue
                pads = [p for p in self.gnd_pads if p.IsOnLayer(layer) and frag.PointInside(p.GetPosition())]
                if any(self.stub(p, layer) for p in pads):
                    done.add(group[i])
            self.fill()
        frags, group = self.groups()
        main = max(set(group), key=group.count)
        for i, (layer, frag) in enumerate(frags):
            if group[i] != main:
                c = frag.BBox().Centre()
                print(f"WARNING: GND pour fragment not tied to the main ground: "
                      f"{self.board.GetLayerName(layer)} ({pcbnew.ToMM(c.x):.2f}, {pcbnew.ToMM(c.y):.2f}) mm")
        for via in list(self.added):
            if not self.in_both(via.GetPosition()):
                self.board.Remove(via)
                self.added.remove(via)
        self.fill()
        print(f"stitched {len(self.added)} GND vias")


def main():
    passes = sys.argv[1] if len(sys.argv) > 1 else "100"
    pcb = os.path.join(HERE, "carrier.kicad_pcb")
    dsn, ses = os.path.join(HERE, "carrier.dsn"), os.path.join(HERE, "carrier.ses")
    board = pcbnew.LoadBoard(pcb)
    # GND pours stay out of the DSN: as planes, Freerouting leaves SMD GND pads stranded on pour
    # fragments; as a plain net it routes every GND pad, and the pours go back in afterwards.
    pours = [z for z in board.Zones() if not z.GetIsRuleArea() and z.GetNetname() == "GND"]
    for z in pours:
        board.Remove(z)
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        sys.exit("DSN export failed")
    subprocess.run([JAVA, "-jar", JAR, "-de", dsn, "-do", ses, "--router.autorouter.max_passes", passes, "--gui.enabled=false"], check=True)
    if not pcbnew.ImportSpecctraSES(board, ses):
        sys.exit("SES import failed")
    for z in pours:
        board.Add(z)
    widen_necks(board)
    Stitcher(board).run()
    board.Save(pcb)
    print(f"routed {pcb}")


if __name__ == "__main__":
    main()
