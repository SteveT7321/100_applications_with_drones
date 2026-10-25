"""Regenerate the per-scenario index in the root README.md.

Each scenario links to its outputs README (results) when it exists, plus its
card and any 3D variant ("-" when there is none). The block between the
SCENARIO-INDEX markers is rewritten in place; run after finishing a scenario:

    python tools/update_readme_index.py
"""
import glob
import io
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
START, END = "<!-- SCENARIO-INDEX:START -->", "<!-- SCENARIO-INDEX:END -->"
DOMAINS = [
    ("01_pursuit_evasion", "Pursuit & Evasion"),
    ("02_logistics_delivery", "Logistics & Delivery"),
    ("03_environmental_sar", "Environmental & SAR"),
    ("04_industrial_agriculture", "Industrial & Agriculture"),
    ("05_special_entertainment", "Special Ops & Entertainment"),
]


def rel(path):
    return os.path.relpath(path, ROOT).replace("\\", "/")


def title_of(card):
    with io.open(card, encoding="utf-8") as f:
        return re.sub(r"^#\s*S\d{3}\s*", "", f.readline()).strip()


def domain_block(num, folder, name):
    cards = sorted(glob.glob(os.path.join(ROOT, "scenarios", folder, "S[0-9][0-9][0-9]_*.md")))
    rows, done = [], 0
    for card in cards:
        sid = os.path.basename(card)[:4]
        out = glob.glob(os.path.join(ROOT, "outputs", folder, sid.lower() + "_*", "README.md"))
        out3d = glob.glob(os.path.join(ROOT, "outputs", folder, "3d", sid.lower() + "_*", "README.md"))
        title = title_of(card)
        if out:
            done += 1
            scenario = f"[{title}]({rel(out[0])})"
        else:
            scenario = f"{title} *(planned)*"
        variant = f"[3D]({rel(out3d[0])})" if out3d else "-"
        rows.append(f"| {sid} | {scenario} | [card]({rel(card)}) | {variant} |")
    return "\n".join([
        f'<a id="domain-{num}"></a>',
        f"<details open><summary><b>{num} · {name}</b> — {done}/{len(cards)} done</summary>",
        "",
        "| ID | Scenario (results) | Card | 3D |",
        "|----|--------------------|------|----|",
        *rows,
        "",
        "</details>",
    ])


def main():
    path = os.path.join(ROOT, "README.md")
    with io.open(path, encoding="utf-8") as f:
        txt = f.read()
    blocks = [domain_block(i + 1, folder, name) for i, (folder, name) in enumerate(DOMAINS)]
    new = START + "\n\n" + "\n\n".join(blocks) + "\n\n" + END
    if START not in txt:
        raise SystemExit(f"{START} marker not found in README.md")
    txt = re.sub(re.escape(START) + r".*?" + re.escape(END), lambda m: new, txt, flags=re.S)
    with io.open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(txt)
    print("README scenario index updated")


if __name__ == "__main__":
    main()
