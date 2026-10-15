# Buck regulator: LMR36520ADDAR (TI), LCSC C2879422

Source: TI SNVSBF0C (Rev C, Nov 2025), sections 6.3, 6.5, 7.4.3, 8.2.1.2. Vout target 5.2 V (first pass used 5.5 V; lowered so the SS34 OR-ing diode leaves ~4.8 V at the Heltec 5V pin, below USB's 5.0 V, so a connected laptop is never back-fed).
JLC: Extended, stock 3372, HSOIC-8 (DDA, exposed pad), ~$1.18. Also stocked: LMR36520FADDAR C1858395 (FPWM variant, do NOT use: no PFM, high Iq).

## Why this part
- Vin 4.2-65 V recommended (abs max higher), SW 70.3 V abs. SMBJ26A clamp (~42 V) has >20 V margin.
- 2 A output (ISC 2.4/3/3.6 A, ILS-LIMIT 1.8/2.3/2.8 A), fixed 400 kHz, internal compensation, no RT pin.
- A-suffix = PFM at light load: IQ non-switching 26 uA typ / 36 uA max (VEN=3.3 V, spec 6.5), shutdown 5.3 uA.
- Pin 5 FB reference VREF = 1.000 V (0.985-1.015). Vout range 1-28 V.
- 5.2 V at Vin 6 V is near dropout (section 7.4.3: frequency folds back to ~140 kHz to raise duty). At 1.5 A the drop is
  roughly 1.5 A x (0.245 + 0.09 DCR) ohm = 0.5 V, so Vout will sag a little below Vin ~5.8 V. Acceptable.

## Component values (Vin 6-36 V nominal, Vout 5.2 V, Iout 1.5 A, device max 2 A)
| Item | Value | Basis |
|---|---|---|
| Switching freq | 400 kHz, fixed internally | Sec 8.2.1.2.1 (no pin to set) |
| RFBT | 1.00 MΩ 1% (C22935, Basic) | Sec 8.2.1.2.2: recommended 100 k, max 1 MΩ with CFF. 1 MΩ chosen so divider adds only 4.2 uA at Vout and keeps no-load Iq low. 100 k would load Vout with 55 uA (~25 uA at Vin 12 V), killing the 30 uA target. |
| RFBB | 240 kΩ 1% (C4197, Preferred) | Eq 3: RFBB = RFBT / (Vout/VREF - 1) = 1M/4.2 = 238 k. E96 237 k is Extended; 240 k is the nearest Basic/Preferred value. 220 k (Basic) would give 5.55 V, which puts H_5V above USB's 5.0 V and defeats the OR diode. |
| Actual Vout | 1.000 x (1 + 1000/240) = 5.17 V (typ; 5.09-5.25 V over VREF +-1.5%, plus resistor tolerance). After the SS34 (~0.3-0.4 V at light load) the Heltec 5V pin sees ~4.75-4.85 V against its 4.7 V minimum: measure on the bench. | Eq 3 solved for Vout |
| CFF (across RFBT) | 10 pF C0G 0603 (C1634, Basic) | Eq 9: CFF = (Vout x Cout) / (120 x RFBT x VREF/Vout) = 5.2 x 44 uF / (120 x 1 M x 0.192) = 10 pF with nominal Cout; ~7 pF if Cout derates to 30 uF. 10 pF. Required because RFBT > 100 k (8.2.1.2.8). Tune on bench. |
| Inductor L | 15 uH | Eq 4: L = (Vin-Vout)/(fsw x K x Iout_max) x Vout/Vin. K=0.4, Iout_max = 2 A (device max, as 8.2.1.2.3 advises when load << device max), Vin=36 V: 30.8/(400k x 0.4 x 2) x 5.2/36 = 13.9 uH -> 15 uH. (TI's own 5 V/400 kHz table 8-3 uses 10 uH at 24 V.) |
| L minimum | 5.8 uH | Eq 5: Lmin = M x Vout/fsw = 0.42 x 5.2/400 kHz. 15 uH passes. |
| Ripple check | dIL = 0.78 A p-p at 36 V; 0.08 A p-p at 6 V | from Eq 4 rearranged; peak at 1.5 A load = 1.9 A < ILS-LIMIT min 1.8 A? see note |
| Inductor part | Sunlord MWSA0605S-150MT, C408468, 15 uH, DCR 90 mΩ, Irms 3.1 A, **Isat 4 A**, 7x6.6 mm, stock 7424 | Sec 8.2.1.2.3: Isat >= high-side limit ISC (max 3.6 A). 4 A meets it. |
| Cin ceramic | 1 x 10 uF 50 V X5R 1206 (C13585, Samsung CL31A106KBHNNNE, Basic) | Sec 8.2.1.2.5: min 4.7 uF. The SMBJ26A clamps VIN at 42.1 V max, so 50 V is enough (TI's example uses 100 V for a 65 V input); about 4 uF effective left at 14 V DC bias. Eq 8: IRMS = Iout/2 = 0.75 A. Was 4.7 uF 100 V 1210 (C381466, Extended). |
| Cin HF bypass | 100 nF 100 V X7R 0805 (C28233, Samsung CL21B104KCFNNNE, Basic) | Sec 8.2.1.2.5 asks for 220 nF as close as possible; 220 nF 100 V is Extended only, so 100 nF Basic. It does the same high-frequency job at 400 kHz; add a second if the bench shows switch-node ringing. |
| Cin damping | 47 uF 50 V SMD aluminum 6.3x7.7 (C970679, DMBJ RVT1H470M0607, stock 38k) | Sec 8.2.1.2.5: electrolytic in parallel damps ringing from long leads. Place after the TVS/fuse, in parallel with ceramics. |
| Cout | 2 x 22 uF 25 V X5R 1206 (C12891, Samsung CL31A226KAHNNNE, Basic) | Table 8-3: 5 V/400 kHz/10 uH uses 2 x 22 uF. 25 V part keeps more capacitance under 5.2 V bias than 10 V parts. Eq 6 sizing is load-step dependent; TI says limit total Cout to <= 10x design value. Add a 100 nF 0603 (C14663) at the output for HF spikes (8.2.1.2.4). |
| CBOOT | 100 nF >= 16 V ceramic, BOOT to SW (C14663, 50 V X7R, Basic) | Sec 8.2.1.2.6 |
| CVCC | 1 uF >= 16 V ceramic, VCC to GND (C15849, 50 V X5R, Basic) | Sec 8.2.1.2.7. VCC (5 V LDO) must not be loaded externally except PG pull-up. |
| EN divider | RENT = 3.00 MΩ (C23156, Preferred) from Vin to EN, RENB = 750 kΩ (C23240, Preferred) EN to GND | Eq 10: RENT = (VON/VEN-H - 1) x RENB with VEN-H = 1.231 V typ (1.157-1.3). Ratio 4.00 (Basic/Preferred values; 3.83 M was Extended): turn-on = 1.231 x 5.00 = 6.16 V (5.79-6.50 V over spread). Turn-off = VON x (1 - 0.110/1.231) = 5.60 V. Divider current only ~3.2 uA at 12 V (EN leakage 2.7 nA). The 10-100 k RENB guidance in 8.2.1.2.9 would burn 10+ uA; 1 MΩ is fine given nA leakage. Optional 1 nF EN-to-GND for noise. |
| EN absolute | EN must not exceed VIN by > 0.3 V (Table 6.3 note 2) - satisfied by divider. | |
| PG (pin 4) | open drain. Leave open (datasheet: "Can be left open when not used") or 100 k to VCC (8.2.1.2.7: 10-100 k). Max 18 V. | |
| MODE/RT | none on this part (fixed 400 kHz, variant A = PFM) | |
| PAD (9) | solder to PGND copper, thermal relief, multiple vias (Table 5-1) | |

Note on the ripple row: Section 6.5 ILS-LIMIT is 1.8-2.8 A and the loop runs valley current limit; peak = Iout + dIL/2 = 1.5 + 0.39 = 1.89 A at 36 V, valley = 1.11 A, well under the valley limit (valley limit is what bounds output current). At 12-16 V input ripple is ~0.3-0.5 A p-p. If you want less ripple at 36 V (never reached from a 12 V car), use 22 uH.

## Estimated no-load input current (Vin 12 V)
IQ 26 uA typ (PFM, regulating at no load; non-switching spec) + FB divider ~4.2 uA at 5.2 V reflected to input (~2 uA at 12 V) + EN divider 3.2 uA = about 31 uA typical, 40 uA worst case. Tight versus the 30 uA target; skipping the EN divider (EN direct to Vin) saves 3.2 uA, and TVS/ceramic leakage is on top. Real PFM Iq with switching at light load is typically higher than the "non-switching" figure, so confirm with a meter.

## Rejected candidates
- LM5164 (C477928, 100 V): only 1 A, rejected per brief (Iq 10.5 uA is great; use it only if load is cut to <=0.9 A).
- LMR38020 (C5149193 "F" 525 kHz PFM / C3192337 "S"): 80 V, 2 A, Iq 40 uA typ. Viable but Iq above LMR36520 (26 uA) and similar price; HSOIC-8 same footprint. Best fallback if more headroom (>65 V) wanted.
- LMR36015 (VQFN-12 2x2, 60 V, 1.5 A): no margin on 1.5 A (hard limit), QFN, stock 15-430.
- TPS54360B (C524806): Iq 146 uA; rejected.
- MP4560 (C40133 $4.42): Iq 140 uA, pricey.
- MP9943 (C477821): 36 V max, Iq 500 uA; fails 45 V.
- SY8303 (C97054): 40 V max, TSOT-23-8; fails 45 V clamp margin.
- XL7015 (C73013): 800 mA, Iq 2.5 mA, asynchronous TO-252; fails.
- LMR33630A (C841384, 36 V, 3 A, Iq 24 uA, 43k stock): 36 V max, fails the 42 V TVS clamp.
- LMR51635 (C42432812, 60 V, 3.5 A, TSOT-23-6): Iq 29 uA, the closest alternative (cheaper footprint; needs checking of its light-load mode: X/Y suffix are FPWM-ish "F" or PFM). Not read in detail; viable second choice.
- TPS62933 (C3200405, SOT-583, 30 V max): fails 45 V.
- AP63205 (C2071056): 32 V max, fails.
