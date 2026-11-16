#!/usr/bin/env python3
"""AKGOZ GIMBAL URDF + mesh ureticisi (yalnizca standart kutuphane).

Olculer iki urun fotografindan oranlanmistir; olcek ornek degerdir:
kulakcik-kulakcik dis genislik ~275 mm kabul edildi. CAD gelince asagidaki
DIM / MASS sozluklerini guncelleyip yeniden calistirin:

    python3 make_akgoz_urdf.py

GEOMETRY (asagida, veya AKGOZ_GEOMETRY ortam degiskeni):
  "cad"   : gimbal-for-fun CAD'i (varsayilan). Gorunus, kutle, COM, atalet tensoru,
            pencere konumlari cad_ref/massprops.json + meshes/cad/*.stl'den.
            Once: <gimbal-for-fun>/.venv/bin/python -I tools/export_ref_cad.py <gimbal-for-fun>/cad/out
  "akgoz" : fotograftan oranlanmis Akgoz gorunusu (DIM/MASS, ornek degerler).

Ciktilar: akgoz_gimbal.urdf, akgoz_inertia.m (+ akgoz modunda meshes/akgoz_hood.stl)
Cerceve: X ileri (optik eksen), Y sol, Z yukari. Gimbal platformun altinda
ters asili durur. Eksenler: azimuth (+Z), elevation (-Y). Roll yoktur.
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
GEOMETRY = os.environ.get("AKGOZ_GEOMETRY", "cad")
assert GEOMETRY in ("cad", "akgoz"), GEOMETRY

# ---------------- Boyutlar [m] ----------------
DIM = dict(
    ground_r=0.20, ground_t=0.02, ground_z=0.05,      # arac govdesi (sabit dunya)
    plate_r=0.14, plate_t=0.015,                      # hareketli platform plakasi
    neck_r=0.056, neck_len=0.112,                     # ust silindir baglanti boynu
    ring_r=0.066, ring_t=0.008, ring_z=-0.075,        # boyun kelepce bileziği
    hood_r_top=0.060, hood_r_bot=0.140, hood_h=0.050, # huni omuz (azimuth govdesi)
    elev_z=-0.277,                                    # elevation ekseni yuksekligi
    ear_r=0.100, ear_t=0.030, ear_y=0.120,            # kulakciklar (Y eksenli disk)
    head_r=0.132,                                     # bas: kure (yanlari kulakciklarin icinde)
    # pencereler: bas merkezine gore (dy, dz)
    ir_dy=0.0145, ir_dz=-0.065, ir_bezel_r=0.058, ir_glass_r=0.046,
    day_dy=0.023, day_dz=0.045, day_bezel_r=0.055, day_glass_r=0.040,
    lrf_dy=-0.078, lrf_dz=0.017, lrf_w=0.066, lrf_h=0.095, lrf_d=0.040,
    lrf_lens_r=0.013, lrf_tx_dz=0.034, lrf_rx_dz=-0.003,
)
# ---------------- Kutleler [kg] (ornek) ----------------
MASS = dict(ground=1.0, plate=1.0, neck=0.6, ring=0.05, hood=0.8, ear=0.35,
            head=2.5, ir_bezel=0.12, ir_glass=0.03, day_bezel=0.08,
            day_glass=0.02, lrf=0.06, lrf_lens=0.005)

# Yatak surtunmesi Simulink'te (akgoz_params.m: Stribeck + viskoz). URDF'te 0.
BEARING_DAMPING = 0.0
# Bas montajinin dengeleme sonrasi kalinti agirlik merkezi ofseti [m], kamera
# cercevesi (X ileri, Z yukari). gimbal-for-fun residual_imbalance [0.5,0,0.3] mm
# FRD -> Z yukari. source: placeholder (olculene kadar).
RESIDUAL_IMBALANCE = (0.0005, 0.0, -0.0003)

GREY = (0.62, 0.66, 0.70, 1)
DARK = (0.20, 0.22, 0.24, 1)


# ---------------- Atalet formulleri ----------------
def cyl(m, r, h, axis):
    """Silindir ataleti; axis: 'x','y','z' silindir ekseni."""
    a = m * r * r / 2
    t = m * (3 * r * r + h * h) / 12
    return dict(x=(a, t, t), y=(t, a, t), z=(t, t, a))[axis]


def box(m, sx, sy, sz):
    return (m * (sy**2 + sz**2) / 12, m * (sx**2 + sz**2) / 12,
            m * (sx**2 + sy**2) / 12)


def frustum(m, r1, r2, h, n=2000):
    """Dolu kesik koni (ust r1 z=0, alt r2 z=-h). Donus: (com_z, ixx, izz)."""
    dz = h / n
    vol = mz = ixx = izz = 0.0
    sl = []
    for i in range(n):
        z = -(i + 0.5) * dz
        r = r1 + (r2 - r1) * (i + 0.5) / n
        dv = math.pi * r * r * dz
        sl.append((z, r, dv))
        vol += dv
        mz += z * dv
    rho = m / vol
    zc = mz / vol
    for z, r, dv in sl:
        dm = rho * dv
        izz += dm * r * r / 2
        ixx += dm * (r * r / 4 + (z - zc) ** 2)
    return zc, ixx, izz


# ---------------- STL ----------------
def write_frustum_stl(path, r1, r2, h, n=96):
    def facet(f, a, b, c):
        ux, uy, uz = (b[i] - a[i] for i in range(3))
        vx, vy, vz = (c[i] - a[i] for i in range(3))
        nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
        L = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        f.write(f" facet normal {nx/L:.6e} {ny/L:.6e} {nz/L:.6e}\n  outer loop\n")
        for p in (a, b, c):
            f.write(f"   vertex {p[0]:.6e} {p[1]:.6e} {p[2]:.6e}\n")
        f.write("  endloop\n endfacet\n")

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("solid akgoz_hood\n")
        top, bot = (0, 0, 0), (0, 0, -h)
        for i in range(n):
            t0, t1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            a0 = (r1 * math.cos(t0), r1 * math.sin(t0), 0)
            a1 = (r1 * math.cos(t1), r1 * math.sin(t1), 0)
            b0 = (r2 * math.cos(t0), r2 * math.sin(t0), -h)
            b1 = (r2 * math.cos(t1), r2 * math.sin(t1), -h)
            facet(f, top, a0, a1)        # ust kapak (+Z)
            facet(f, bot, b1, b0)        # alt kapak (-Z)
            facet(f, a0, b0, b1)         # yan yuzey
            facet(f, a0, b1, a1)
        f.write("endsolid akgoz_hood\n")


# ---------------- URDF yazici ----------------
out = []


def fmt(v):
    return " ".join(f"{x:.10g}" for x in v)


def link(name, m, inertia, geom=None, color=None, xyz=(0, 0, 0), rpy=(0, 0, 0),
         com=None, offdiag=(0.0, 0.0, 0.0)):
    com = xyz if com is None else com
    ixx, iyy, izz = inertia
    ixy, ixz, iyz = offdiag
    s = [f'  <link name="{name}">',
         '    <inertial>',
         f'      <origin xyz="{fmt(com)}" rpy="0 0 0" />',
         f'      <mass value="{m:.10g}" />',
         f'      <inertia ixx="{ixx:.10g}" iyy="{iyy:.10g}" izz="{izz:.10g}" ixy="{ixy:.10g}" ixz="{ixz:.10g}" iyz="{iyz:.10g}" />',
         '    </inertial>']
    if geom:
        s += ['    <visual>',
              f'      <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}" />',
              f'      <geometry>{geom}</geometry>',
              f'      <material name="{name}_material"><color rgba="{fmt(color)}" /></material>',
              '    </visual>']
    s.append('  </link>')
    out.append("\n".join(s))


def joint(name, typ, parent, child, xyz=(0, 0, 0), axis=None, damping=None):
    s = [f'  <joint name="{name}" type="{typ}">',
         f'    <parent link="{parent}" />', f'    <child link="{child}" />',
         f'    <origin xyz="{fmt(xyz)}" rpy="0 0 0" />']
    if axis:
        s.append(f'    <axis xyz="{fmt(axis)}" />')
    if damping:
        s.append(f'    <dynamics damping="{damping:.10g}" friction="0" />')
    s.append('  </joint>')
    out.append("\n".join(s))


def C(r, l):
    return f'<cylinder radius="{r:.10g}" length="{l:.10g}" />'


def write_urdf(comment):
    hdr = ("<?xml version='1.0' encoding='utf-8'?>\n"
           "<!-- AKGOZ GIMBAL - 2 eksen (azimuth + elevation), roll yok.\n"
           "     make_akgoz_urdf.py tarafindan uretildi; elle duzenlemeyin.\n"
           f"     {comment} -->\n"
           '<robot name="akgoz_gimbal">\n')
    with open(os.path.join(HERE, "akgoz_gimbal.urdf"), "w") as f:
        f.write(hdr + "\n".join(out) + "\n</robot>\n")


def write_inertia_m(J, src):
    with open(os.path.join(HERE, "akgoz_inertia.m"), "w") as f:
        f.write("function J = akgoz_inertia()\n"
                "% make_akgoz_urdf.py tarafindan uretildi; elle duzenlemeyin. [kg*m^2]\n"
                f"% source: {src}\n"
                "% Tensors in FLU (x fwd, y left, z up) about O (axis intersection).\n"
                f"J.el      = {J['el']:.10g};  % head (pitch body), elevation axis\n"
                f"J.head_xx = {J['head_xx']:.10g};  % head tensor, camera frame\n"
                f"J.head_zz = {J['head_zz']:.10g};\n"
                f"J.head_xz = {J['head_xz']:.10g};\n"
                f"J.yoke_zz = {J['yoke_zz']:.10g};  % yoke (yaw body), azimuth axis\n"
                f"J.m_head  = {J['m_head']:.10g};  % head mass [kg]\n"
                "% J_az(el) = yoke_zz + head_xx*sin(el)^2 + head_zz*cos(el)^2 + 2*head_xz*sin(el)*cos(el)\n"
                "end\n")


def build_cad():
    """gimbal-for-fun turret: inertials from cad_ref/massprops.json, visuals from meshes/cad."""
    mp = json.load(open(os.path.join(HERE, "cad_ref", "massprops.json")))
    man = json.load(open(os.path.join(HERE, "meshes", "cad", "manifest.json")))
    T = (1.0, -1.0, -1.0)                                   # FRD -> FLU

    def flu_vec(v):
        return tuple(T[i] * v[i] for i in range(3))

    def flu_tensor(I):
        return [[T[i] * T[j] * I[i][j] for j in range(3)] for i in range(3)]

    def about_com(I_O, m, c):                               # parallel axis: O -> COM
        cc = sum(x * x for x in c)
        return [[I_O[i][j] - m * ((cc if i == j else 0.0) - c[i] * c[j]) for j in range(3)] for i in range(3)]

    zO = mp["geometry_mm"]["z_mount"] / 1000.0              # FRD z of mount plane (negative = above O)
    B = {}
    for name, b in mp["bodies"].items():
        m = b["mass_kg"]
        c = flu_vec(b["com_m"])
        I_O = flu_tensor(b["inertia_O_kgm2"])
        B[name] = dict(m=m, c=c, I_O=I_O)
    # residual imbalance on the head (FRD in gimbal-for-fun params; placeholder)
    ri = flu_vec((0.0005, 0.0, 0.0003))
    hp = B["pitch_body"]
    hp["c"] = tuple(hp["c"][i] + ri[i] for i in range(3))

    def inertial(body, shift=(0.0, 0.0, 0.0)):
        b = B[body]
        Ic = about_com(b["I_O"], b["m"], b["c"])
        com = tuple(b["c"][i] + shift[i] for i in range(3))
        return b["m"], (Ic[0][0], Ic[1][1], Ic[2][2]), (Ic[0][1], Ic[0][2], Ic[1][2]), com

    link("ground", 1.0, cyl(1.0, 0.2, 0.02, "z"), C(0.2, 0.02), (0.25, 0.27, 0.30, 1), xyz=(0, 0, 0.05))
    link("yaw_carrier", 0.001, (1.6667e-08,) * 3)
    joint("base_yaw", "continuous", "ground", "yaw_carrier", axis=(0, 0, 1))
    # moving_base frame = mount plane; O lies zO (FRD z_mount) along FLU z: FLU z of O = z_mount (negative)
    m, I, od, com = inertial("base", (0.0, 0.0, zO))
    link("moving_base", m, I, com=com, offdiag=od)
    joint("base_pitch", "continuous", "yaw_carrier", "moving_base", axis=(0, 1, 0))
    m, I, od, com = inertial("yaw_body")
    link("azimuth_body", m, I, com=com, offdiag=od)
    joint("azimuth", "continuous", "moving_base", "azimuth_body", xyz=(0, 0, zO), axis=(0, 0, 1))
    m, I, od, com = inertial("pitch_body")
    link("camera", m, I, com=com, offdiag=od)
    joint("elevation", "continuous", "azimuth_body", "camera", axis=(0, -1, 0))
    parent = dict(base="moving_base", yaw_body="azimuth_body", pitch_body="camera")
    for k, msh in enumerate(man["meshes"]):
        nm = os.path.splitext(os.path.basename(msh["file"]))[0] + "_vis"
        off = (0.0, 0.0, zO) if msh["body"] == "base" else (0.0, 0.0, 0.0)
        link(nm, 1e-6, (1e-12,) * 3, f'<mesh filename="{msh["file"]}" scale="1 1 1" />',
             tuple(msh["rgb"]) + (1,), xyz=off, com=(0, 0, 0))
        joint(nm + "_mount", "fixed", parent[msh["body"]], nm)
    hI = B["pitch_body"]["I_O"]
    write_inertia_m(dict(el=hI[1][1], head_xx=hI[0][0], head_zz=hI[2][2], head_xz=hI[0][2],
                         yoke_zz=B["yaw_body"]["I_O"][2][2], m_head=B["pitch_body"]["m"]),
                    "cad:gimbal-for-fun cad/turret.py (cad_ref/massprops.json)")
    write_urdf("Geometri ve kutle: gimbal-for-fun cad/turret.py (cad_ref/massprops.json, meshes/cad).")
    g = mp["geometry_mm"]
    print(f"CAD: toplam {mp['total_mass_kg']:.3f} kg, top D {2*g['r_out']:.0f} mm, O montajin {-zO*1000:.0f} mm altinda")
    print(f"J_el={hI[1][1]:.5f}  J_az(0)={B['yaw_body']['I_O'][2][2] + hI[2][2]:.5f} kg*m^2 -> akgoz_inertia.m")
    print("akgoz_gimbal.urdf yazildi (GEOMETRY=cad).")


if GEOMETRY == "cad":
    build_cad()
    raise SystemExit(0)

D, M = DIM, MASS
RX = (0, math.pi / 2, 0)   # silindiri X eksenine yatir
RY = (math.pi / 2, 0, 0)   # silindiri Y eksenine yatir

# Arac / taban bozucu zinciri (yaw, pitch)
link("ground", M["ground"], cyl(M["ground"], D["ground_r"], D["ground_t"], "z"),
     C(D["ground_r"], D["ground_t"]), (0.25, 0.27, 0.30, 1), xyz=(0, 0, D["ground_z"]))
link("yaw_carrier", 0.001, (1.6667e-08,) * 3)
link("moving_base", M["plate"], cyl(M["plate"], D["plate_r"], D["plate_t"], "z"),
     C(D["plate_r"], D["plate_t"]), (0.25, 0.45, 0.70, 1))
joint("base_yaw", "continuous", "ground", "yaw_carrier", axis=(0, 0, 1))
joint("base_pitch", "continuous", "yaw_carrier", "moving_base", axis=(0, 1, 0))

# Sabit ust boyun ve bilezik
z_neck_top = -D["plate_t"] / 2
neck_c = z_neck_top - D["neck_len"] / 2
link("mount_neck", M["neck"], cyl(M["neck"], D["neck_r"], D["neck_len"], "z"),
     C(D["neck_r"], D["neck_len"]), GREY)
joint("neck_mount", "fixed", "moving_base", "mount_neck", xyz=(0, 0, neck_c))
link("neck_ring", M["ring"], cyl(M["ring"], D["ring_r"], D["ring_t"], "z"),
     C(D["ring_r"], D["ring_t"]), GREY)
joint("ring_mount", "fixed", "moving_base", "neck_ring", xyz=(0, 0, D["ring_z"]))

# Azimuth govdesi: huni omuz (STL). Link cercevesi = boyun alti = azimuth ekseni.
z_az = z_neck_top - D["neck_len"]
zc, hixx, hizz = frustum(M["hood"], D["hood_r_top"], D["hood_r_bot"], D["hood_h"])
write_frustum_stl(os.path.join(HERE, "meshes", "akgoz_hood.stl"),
                  D["hood_r_top"], D["hood_r_bot"], D["hood_h"])
link("azimuth_body", M["hood"], (hixx, hixx, hizz),
     '<mesh filename="meshes/akgoz_hood.stl" scale="1 1 1" />', GREY,
     com=(0, 0, zc))
joint("azimuth", "continuous", "moving_base", "azimuth_body", xyz=(0, 0, z_az),
      axis=(0, 0, 1), damping=BEARING_DAMPING)

# Kulakciklar: azimuth govdesine sabit, Y eksenli yuvarlak diskler
e_rel = D["elev_z"] - z_az     # elevation ekseninin azimuth cercevesindeki z'si
for side, sgn in (("left", 1), ("right", -1)):
    link(f"{side}_ear", M["ear"], cyl(M["ear"], D["ear_r"], D["ear_t"], "y"),
         C(D["ear_r"], D["ear_t"]), GREY, rpy=RY)
    joint(f"{side}_ear_mount", "fixed", "azimuth_body", f"{side}_ear",
          xyz=(0, sgn * D["ear_y"], e_rel))

# On parcalar (bas cercevesinde): (isim, kutle, atalet, geom, renk, xyz, rpy)
R = D["head_r"]
xs = lambda dy, dz: math.sqrt(R * R - dy * dy - dz * dz)   # kure on yuzeyi x
front = []
x = xs(D["ir_dy"], D["ir_dz"])
front += [("ir_bezel", M["ir_bezel"], cyl(M["ir_bezel"], D["ir_bezel_r"], 0.032, "x"),
           C(D["ir_bezel_r"], 0.032), GREY, (x - 0.008, D["ir_dy"], D["ir_dz"]), RX),
          ("ir_window", M["ir_glass"], cyl(M["ir_glass"], D["ir_glass_r"], 0.004, "x"),
           C(D["ir_glass_r"], 0.004), (0.55, 0.35, 0.62, 1),
           (x + 0.010, D["ir_dy"], D["ir_dz"]), RX)]
x = xs(D["day_dy"], D["day_dz"])
front += [("day_bezel", M["day_bezel"], cyl(M["day_bezel"], D["day_bezel_r"], 0.05, "x"),
           C(D["day_bezel_r"], 0.05), GREY, (x - 0.017, D["day_dy"], D["day_dz"]), RX),
          ("day_window", M["day_glass"], cyl(M["day_glass"], D["day_glass_r"], 0.004, "x"),
           C(D["day_glass_r"], 0.004), (0.78, 0.62, 0.15, 1),
           (x + 0.010, D["day_dy"], D["day_dz"]), RX)]
x = xs(D["lrf_dy"], D["lrf_dz"])
front += [("lrf_housing", M["lrf"], box(M["lrf"], D["lrf_d"], D["lrf_w"], D["lrf_h"]),
           f'<box size="{D["lrf_d"]} {D["lrf_w"]} {D["lrf_h"]}" />', DARK,
           (x - 0.014, D["lrf_dy"], D["lrf_dz"]), (0, 0, 0))]
for nm, dz, col in (("lrf_tx", D["lrf_tx_dz"], (0.75, 0.25, 0.30, 1)),
                    ("lrf_rx", D["lrf_rx_dz"], (0.20, 0.60, 0.40, 1))):
    front.append((nm, M["lrf_lens"], cyl(M["lrf_lens"], D["lrf_lens_r"], 0.004, "x"),
                  C(D["lrf_lens_r"], 0.004), col,
                  (x + 0.008, D["lrf_dy"], dz), RX))   # dz: bas merkezine gore

# Elevation dengesi: on parcalarin momentini bas davulunun agirlik merkezini
# kaydirarak sifirla (gercek gimballer dengelenir; dengesizlik yercekimi torku yapar).
mx = sum(m * p[0] for _, m, _, _, _, p, _ in front)
mz = sum(m * p[2] for _, m, _, _, _, p, _ in front)
m_front = sum(m for _, m, _, _, _, _, _ in front)
m_tot = M["head"] + m_front
ri = RESIDUAL_IMBALANCE
head_com = ((m_tot * ri[0] - mx) / M["head"], m_tot * ri[1] / M["head"],
            (m_tot * ri[2] - mz) / M["head"])

link("camera", M["head"], (0.4 * M["head"] * R * R,) * 3,
     f'<sphere radius="{R:.10g}" />', GREY, com=head_com)
joint("elevation", "continuous", "azimuth_body", "camera", xyz=(0, 0, e_rel),
      axis=(0, -1, 0), damping=BEARING_DAMPING)
for n, m, I, g, c, p, r in front:
    link(n, m, I, g, c, rpy=r)
    joint(f"{n}_mount", "fixed", "camera", n, xyz=p)

hdr = ("<?xml version='1.0' encoding='utf-8'?>\n"
       "<!-- AKGOZ GIMBAL - 2 eksen (azimuth + elevation), roll yok.\n"
       "     make_akgoz_urdf.py tarafindan uretildi; elle duzenlemeyin.\n"
       "     Olculer fotograftan oranlanmis ORNEK degerlerdir. -->\n"
       '<robot name="akgoz_gimbal">\n')
with open(os.path.join(HERE, "akgoz_gimbal.urdf"), "w") as f:
    f.write(hdr + "\n".join(out) + "\n</robot>\n")


# ---------------- Kontrolcu icin eksen ataletleri ----------------
def add_tensor(T, m, Ilocal, r):
    rr = sum(x * x for x in r)
    for i in range(3):
        for j in range(3):
            T[i][j] += (Ilocal[i] if i == j else 0.0) + m * ((rr if i == j else 0.0) - r[i] * r[j])


TH = [[0.0] * 3 for _ in range(3)]      # bas montaji, kamera cercevesi, elevation merkezi
add_tensor(TH, M["head"], (0.4 * M["head"] * R * R,) * 3, head_com)
for _, m, I, _, _, p, _ in front:
    add_tensor(TH, m, I, p)
TY = [[0.0] * 3 for _ in range(3)]      # yoke (omuz + kulakciklar), azimuth cercevesi
add_tensor(TY, M["hood"], (hixx, hixx, hizz), (0, 0, zc))
for sgn in (1, -1):
    add_tensor(TY, M["ear"], cyl(M["ear"], D["ear_r"], D["ear_t"], "y"), (0, sgn * D["ear_y"], e_rel))
with open(os.path.join(HERE, "akgoz_inertia.m"), "w") as f:
    f.write("function J = akgoz_inertia()\n"
            "% make_akgoz_urdf.py tarafindan uretildi; elle duzenlemeyin. [kg*m^2]\n"
            "% source: calc:make_akgoz_urdf.py (fotograftan oranlanmis ornek geometri)\n"
            f"J.el      = {TH[1][1]:.10g};  % bas montaji, elevation ekseni\n"
            f"J.head_xx = {TH[0][0]:.10g};  % bas montaji tensoru, kamera cercevesi\n"
            f"J.head_zz = {TH[2][2]:.10g};\n"
            f"J.head_xz = {TH[0][2]:.10g};\n"
            f"J.yoke_zz = {TY[2][2]:.10g};  % omuz + kulakciklar, azimuth ekseni\n"
            f"J.m_head  = {m_tot:.10g};  % bas montaji kutlesi [kg]\n"
            "% J_az(el) = yoke_zz + head_xx*sin(el)^2 + head_zz*cos(el)^2 + 2*head_xz*sin(el)*cos(el)\n"
            "end\n")
print(f"J_el={TH[1][1]:.5f}  J_az(0)={TY[2][2] + TH[2][2]:.5f} kg*m^2 -> akgoz_inertia.m")
print("akgoz_gimbal.urdf ve meshes/akgoz_hood.stl yazildi.")
print(f"elevation z={D['elev_z']:.3f} m, bas COM ofseti={head_com}")
