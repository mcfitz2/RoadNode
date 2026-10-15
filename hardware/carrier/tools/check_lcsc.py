"""Checks every LCSC code in carrier.net against JLCPCB's parts search.

Prints one line per distinct code: refs, netlist value/footprint, and what JLC
says the part is (model, package, Basic/Extended, stock). Review by eye:
a guessed code that points at the wrong value or package shows up here.

    python3 tools/check_lcsc.py [carrier.net]
"""
import json
import re
import sys
import time
import urllib.request

API = "https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList"


def lookup(code):
    body = json.dumps({"currentPage": 1, "pageSize": 5, "keyword": code, "searchSource": "search"}).encode()
    req = urllib.request.Request(API, body, {"Content-Type": "application/json"})
    data = json.load(urllib.request.urlopen(req, timeout=30))
    for c in (data["data"]["componentPageInfo"] or {}).get("list") or []:
        if c["componentCode"] == code:
            return c
    return None


def parts(netlist):
    text = open(netlist).read().split("(nets")[0]
    for comp in re.split(r"\(comp\s", text)[1:]:
        ref = re.search(r'\(ref "([^"]+)"\)', comp).group(1)
        value = re.search(r'\(value "([^"]*)"\)', comp).group(1)
        fp = re.search(r'\(footprint "([^"]*)"\)', comp)
        lcsc = re.search(r'\(name "LCSC"\) "([^"]*)"\)', comp)
        yield ref, value, fp.group(1) if fp else "", lcsc.group(1) if lcsc else ""


def main():
    by_code = {}
    for ref, value, fp, code in parts(sys.argv[1] if len(sys.argv) > 1 else "carrier.net"):
        if not code:
            print(f"{ref:6} {value:16} not assembled (hand-fit)")
            continue
        by_code.setdefault(code, []).append((ref, value, fp))
    bad = 0
    for code, uses in sorted(by_code.items()):
        refs = ",".join(u[0] for u in uses)
        value, fp = uses[0][1], uses[0][2].split(":")[-1]
        c = lookup(code)
        time.sleep(0.5)
        if not c:
            bad += 1
            print(f"{code:10} {refs:24} {value:16} {fp:34} NOT FOUND")
            continue
        flag = "" if c["stockCount"] > 100 else "  LOW STOCK"
        print(f"{code:10} {refs:24} {value:16} {fp:34} -> {c['componentLibraryType']:8} {c['stockCount']:>8} "
              f"{c['componentModelEn'][:26]:26} {c.get('componentSpecificationEn', '')[:12]:12} "
              f"{c.get('describe', '')[:70]}{flag}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
