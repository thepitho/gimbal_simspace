#!/usr/bin/env python3
"""Export gimbal-for-fun CAD outputs for the AKGOZ URDF (needs numpy).

    <gimbal-for-fun>/.venv/bin/python -I tools/export_ref_cad.py <gimbal-for-fun>/cad/out

Reads  {pitch_body,yaw_body,base}.npz (triangle soup, m, FRD about O) + massprops.json.
Writes meshes/cad/<body>_<k>.stl (binary, one file per vertex colour, converted to
FLU = x fwd, y left, z up, still about O), meshes/cad/manifest.json and
cad_ref/massprops.json (verbatim copy, source of the URDF inertials).
"""
import json
import shutil
import struct
import sys
from pathlib import Path

import numpy as np

src = Path(sys.argv[1]).expanduser().resolve()
here = Path(__file__).resolve().parent.parent
mdir = here / "meshes" / "cad"
mdir.mkdir(parents=True, exist_ok=True)
(here / "cad_ref").mkdir(exist_ok=True)
shutil.copy2(src / "massprops.json", here / "cad_ref" / "massprops.json")

manifest = []
for body in ("base", "yaw_body", "pitch_body"):
    z = np.load(src / f"{body}.npz")
    pos = z["pos"].astype(np.float64).reshape(-1, 3, 3)
    col = z["col"].astype(np.float64).reshape(-1, 3, 3)[:, 0, :]
    pos = pos * np.array([1.0, -1.0, -1.0])            # FRD -> FLU (180 deg about x, winding kept)
    keys = np.round(col, 3)
    uniq = np.unique(keys, axis=0)
    for k, c in enumerate(uniq):
        tri = pos[np.all(keys == c, axis=1)]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-30)
        name = f"{body}_{k}.stl"
        with open(mdir / name, "wb") as f:
            f.write(b"gimbal-for-fun cad export (FLU, about O)".ljust(80, b" "))
            f.write(struct.pack("<I", len(tri)))
            for t, nn in zip(tri.astype(np.float32), n.astype(np.float32)):
                f.write(struct.pack("<12fH", *nn, *t.reshape(-1), 0))
        manifest.append(dict(file=f"meshes/cad/{name}", body=body, rgb=[float(x) for x in c],
                             triangles=int(len(tri))))
with open(mdir / "manifest.json", "w") as f:
    json.dump(dict(source=str(src), frame="FLU about O (axis intersection)", meshes=manifest), f, indent=1)
print(f"{len(manifest)} STL files -> {mdir}")
