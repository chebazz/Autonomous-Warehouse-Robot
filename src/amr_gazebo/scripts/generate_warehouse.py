#!/usr/bin/env python3
"""Generador del mundo de Gazebo "warehouse" (depósito industrial detallado).

El SDF resultante se escribe en worlds/warehouse.sdf. Todas las cotas están en
las constantes de la sección LAYOUT, así que el depósito se puede rehacer
cambiando parámetros y volviendo a ejecutar:

    python3 generate_warehouse.py                 # escribe ../worlds/warehouse.sdf
    python3 generate_warehouse.py --plot plano.png  # además dibuja el plano

Distribución (vista superior, x hacia la derecha, y hacia arriba; 40 x 24 m):

    y=+12 +------------------------------------------------------------+
          | RACK A (simple)               |   | BLOQUE suelo | OFICINA  |
          |  ··· pasillo 1 ···            | P |              |          |
          | RACK B (doble)                | A |  col     col  CARGA AMR |
          |  ··· pasillo 2 ···            | S |               CARRET.   |
          | RACK C (doble)        (0,0)-> | I |                         |
          |  ··· pasillo 3 ···            | L |  col     col            |
          | RACK D (simple)               | L |                CINTA    |
          |================= ZONA DE PREPARACIÓN / MUELLES ===========   |
          |  [EST1]   [EST2]   [EST3]   [EST4]        carretilla      ...|
    y=-12 +===D1=======D2=======D3=======D4==============================+
          x=-20                                                     x=+20

Convenciones:
  - Modelos estáticos con un único link y muchas visuales (barato de simular).
  - Colisiones sólo en lo que el robot puede tocar a ras de suelo; los niveles
    altos de las estanterías son únicamente visuales (los LiDAR usan visuales).
  - Los palets de las estaciones son dinámicos: el AMR puede elevarlos.
"""
import argparse
import math
import os
import random

# ============================================================================
# LAYOUT
# ============================================================================
SEED = 7

HALL_X = (-20.0, 20.0)       # caras interiores de los muros
HALL_Y = (-12.0, 12.0)
WALL_T = 0.30
WALL_H = 8.0

# --- Estanterías de palets (racks convencionales) ---------------------------
RACK_X0 = -19.4              # inicio de las filas (extremo oeste)
BAY_W = 2.7                  # ancho de vano entre bastidores
N_BAYS = 6
RACK_D = 1.1                 # fondo de un rack simple
FLUE = 0.2                   # separación entre racks espalda con espalda
UPRIGHT_H = 6.0
BEAM_LEVELS = [1.5, 3.0, 4.5]          # altura (centro) de los largueros
BEAM_H = 0.12
SLOTS_PER_BAY = 3
RACK_FILL = 0.72                       # probabilidad de hueco ocupado

# Filas: (nombre, y_min de la fila, número de racks 1=simple, 2=doble)
RACK_ROWS = [
    ('rack_A', 10.6, 1),
    ('rack_B', 4.8, 2),
    ('rack_C', -1.0, 2),
    ('rack_D', -5.5, 1),
]
AISLES_Y = [(7.2, 10.6), (1.4, 4.8), (-4.4, -1.0)]   # pasillos (y_min, y_max)

# --- Zona de preparación / muelles -----------------------------------------
STAGING_Y = (-12.0, -6.2)
DOCK_DOORS_X = [-14.0, -9.0, -4.0, 1.0]
DOCK_DOOR_W = 3.0
DOCK_DOOR_H = 3.6
# Estaciones de palet. Marco local: ménsulas en x = ±0.6 (apoyan los extremos
# cortos del palet); el AMR entra por el lado +y local mirando hacia -y local.
#   yaw = 0   → entrada desde el norte     yaw = pi  → desde el sur
#   yaw = pi/2 → desde el oeste            yaw = -pi/2 → desde el este
# cargo_kg: palet dinámico cargado sobre la estación (None = vacía).
APPROACH_DIST = 1.7   # distancia centro estación → pose de pre-atraque [m]
STATIONS = [
    # Área de carga (muelles): palets que llegan de los camiones. y = -9.0 deja
    # un pasillo de ~3 m frente al rack D para circular con palet.
    dict(id='P1', area='carga', x=-14.0, y=-9.0, yaw=0.0, cargo_kg=800.0),
    dict(id='P2', area='carga', x=-9.0, y=-9.0, yaw=0.0, cargo_kg=650.0),
    dict(id='P3', area='carga', x=-4.0, y=-9.0, yaw=0.0, cargo_kg=500.0),
    dict(id='P4', area='carga', x=1.0, y=-9.0, yaw=0.0, cargo_kg=700.0),
    # Áreas de destino (zona este)
    dict(id='A1-1', area='area_1', x=7.5, y=2.0, yaw=math.pi, cargo_kg=None),
    dict(id='A1-2', area='area_1', x=10.0, y=2.0, yaw=math.pi, cargo_kg=None),
    dict(id='A2-1', area='area_2', x=7.5, y=-4.0, yaw=0.0, cargo_kg=None),
    dict(id='A2-2', area='area_2', x=10.0, y=-4.0, yaw=0.0, cargo_kg=None),
    dict(id='A3-1', area='area_3', x=16.0, y=-1.0, yaw=math.pi / 2, cargo_kg=None),
    dict(id='A3-2', area='area_3', x=16.0, y=-3.5, yaw=math.pi / 2, cargo_kg=None),
]
AREA_COLORS = {'carga': 'yellow', 'area_1': (0.10, 0.40, 0.90),
               'area_2': (0.60, 0.20, 0.80), 'area_3': (0.00, 0.70, 0.75)}


def station_to_world(st, lx, ly):
    c, s_ = math.cos(st['yaw']), math.sin(st['yaw'])
    return st['x'] + c * lx - s_ * ly, st['y'] + s_ * lx + c * ly


def station_poses(st):
    """Pose de pre-atraque y de atraque (x, y, yaw del robot) de una estación."""
    robot_yaw = math.atan2(math.sin(st['yaw'] - math.pi / 2), math.cos(st['yaw'] - math.pi / 2))
    px, py = station_to_world(st, 0.0, APPROACH_DIST)
    return (px, py, robot_yaw), (st['x'], st['y'], robot_yaw)

# --- Zona este -------------------------------------------------------------
WALKWAY_X = (1.0, 2.0)                 # pasarela peatonal (verde)
COLUMNS = [(5.0, -2.0), (5.0, 6.0), (12.5, -2.0), (12.5, 6.0)]
OFFICE = (14.0, 20.0, 6.5, 12.0)       # x0, x1, y0, y1
OFFICE_DOOR_X = (15.0, 16.0)
BLOCK_STORAGE = (2.8, 9.8, 2)          # x0, y_fila_exterior.. (ver block_storage())
CHARGERS_Y = [1.0, 3.0]
CONVEYOR = (18.9, -10.5, -4.5)         # x centro, y0, y1
FORKLIFT = (9.5, -9.6, 0.0)            # x, y, yaw
PEOPLE = [  # (nombre, x, y, yaw)
    ('person_1', 1.5, -3.0, math.pi / 2),
    ('person_2', -12.0, -2.7, 0.0),
    ('person_3', 15.5, 5.2, -math.pi / 2),
]

# ============================================================================
# COLORES
# ============================================================================
C = {
    'floor': (0.52, 0.53, 0.55),
    'wall': (0.82, 0.83, 0.80),
    'wall_band': (0.25, 0.42, 0.62),
    'upright': (0.10, 0.28, 0.62),
    'beam': (0.95, 0.42, 0.05),
    'wood': (0.72, 0.55, 0.32),
    'wood_dark': (0.55, 0.40, 0.22),
    'cardboard': (0.66, 0.50, 0.32),
    'wrap': (0.85, 0.88, 0.92),
    'crate_blue': (0.15, 0.35, 0.65),
    'crate_green': (0.18, 0.48, 0.25),
    'crate_grey': (0.40, 0.42, 0.45),
    'yellow': (0.95, 0.78, 0.05),
    'white': (0.95, 0.95, 0.95),
    'green': (0.10, 0.62, 0.25),
    'black': (0.08, 0.08, 0.08),
    'red': (0.80, 0.10, 0.10),
    'concrete': (0.68, 0.68, 0.66),
    'steel': (0.62, 0.64, 0.68),
    'door': (0.55, 0.60, 0.66),
    'glass': (0.55, 0.75, 0.90),
    'orange': (0.95, 0.45, 0.05),
    'skin': (0.85, 0.68, 0.55),
    'hivis': (0.80, 0.95, 0.10),
    'navy': (0.12, 0.15, 0.30),
}
CARGO_COLORS = ['cardboard', 'cardboard', 'wrap', 'wrap', 'crate_blue', 'crate_green', 'crate_grey']

PLOT_ITEMS = []   # (x, y, sx, sy, yaw, color, z_top) para el plano

# Las cajas opacas de los modelos estáticos se fusionan en una malla OBJ por
# modelo (un submesh por material). Reduce miles de draw calls a unas decenas:
# imprescindible para que los LiDAR (gpu_lidar) rendericen en tiempo real.
MESH_DIR = None                                   # se fija en main()
MESH_URI = 'model://amr_gazebo/worlds/meshes'     # resuelto vía GZ_SIM_RESOURCE_PATH


# ============================================================================
# PRIMITIVAS SDF
# ============================================================================
def _f(v):
    return f'{v:.4f}'.rstrip('0').rstrip('.') if isinstance(v, float) else str(v)


def _pose(p):
    p = list(p) + [0.0] * (6 - len(p))
    return ' '.join(_f(float(v)) for v in p)


def _size(s):
    return ' '.join(_f(float(v)) for v in s)


def _material(color, alpha=1.0, emissive=None):
    c = C[color] if isinstance(color, str) else color
    rgba = f'{c[0]} {c[1]} {c[2]} {alpha}'
    em = f'<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>' if emissive else ''
    return (f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular>{em}</material>')


class Link:
    """Acumula visuales/colisiones de un link."""

    def __init__(self, name='link', origin=(0.0, 0.0, 0.0, 0.0), merge=True):
        self.name = name
        self.items = []
        self.n = 0
        self.origin = origin   # (x, y, z, yaw) del modelo, sólo para el plano
        self.merge = merge     # fusionar cajas opacas en una malla
        self.mesh_boxes = []   # (pose, size, color)

    def _uid(self, prefix):
        self.n += 1
        return f'{prefix}_{self.n}'

    def _plot(self, pose, sx, sy, sz, color):
        ox, oy, oz, oyaw = self.origin
        x, y, z = pose[0], pose[1], pose[2]
        yaw = pose[5] if len(pose) > 5 else 0.0
        wx = ox + x * math.cos(oyaw) - y * math.sin(oyaw)
        wy = oy + x * math.sin(oyaw) + y * math.cos(oyaw)
        PLOT_ITEMS.append((wx, wy, sx, sy, oyaw + yaw, color, oz + z + sz / 2.0))

    def box(self, pose, size, color, collide=False, visual=True, alpha=1.0,
            emissive=None, plot=True):
        if visual and self.merge and alpha >= 1.0 and emissive is None:
            self.mesh_boxes.append((pose, size, color))
            if plot:
                self._plot(pose, size[0], size[1], size[2], color)
        elif visual:
            self.items.append(
                f'<visual name="{self._uid("v")}"><pose>{_pose(pose)}</pose>'
                f'<geometry><box><size>{_size(size)}</size></box></geometry>'
                f'{_material(color, alpha, emissive)}</visual>')
            if plot:
                self._plot(pose, size[0], size[1], size[2], color)
        if collide:
            self.items.append(
                f'<collision name="{self._uid("c")}"><pose>{_pose(pose)}</pose>'
                f'<geometry><box><size>{_size(size)}</size></box></geometry></collision>')

    def cylinder(self, pose, radius, length, color, collide=False):
        self.items.append(
            f'<visual name="{self._uid("v")}"><pose>{_pose(pose)}</pose>'
            f'<geometry><cylinder><radius>{_f(radius)}</radius><length>{_f(length)}</length>'
            f'</cylinder></geometry>{_material(color)}</visual>')
        self._plot(pose, 2 * radius, 2 * radius, length, color)
        if collide:
            self.items.append(
                f'<collision name="{self._uid("c")}"><pose>{_pose(pose)}</pose>'
                f'<geometry><cylinder><radius>{_f(radius)}</radius><length>{_f(length)}</length>'
                f'</cylinder></geometry></collision>')

    def sphere(self, pose, radius, color):
        self.items.append(
            f'<visual name="{self._uid("v")}"><pose>{_pose(pose)}</pose>'
            f'<geometry><sphere><radius>{_f(radius)}</radius></sphere></geometry>'
            f'{_material(color)}</visual>')

    def marking(self, x, y, sx, sy, color, yaw=0.0):
        """Pintura en el suelo (sin colisión)."""
        self.box((x, y, 0.001, 0, 0, yaw), (sx, sy, 0.002), color)

    def outline(self, x, y, sx, sy, color, lw=0.1):
        """Recuadro pintado en el suelo."""
        for (dx, dy, a, b) in [(0, sy / 2, sx + lw, lw), (0, -sy / 2, sx + lw, lw),
                               (sx / 2, 0, lw, sy), (-sx / 2, 0, lw, sy)]:
            self.marking(x + dx, y + dy, a, b, color)

    def sdf(self, inertial='', mesh_name=None):
        mesh = ''
        if self.mesh_boxes:
            write_obj(os.path.join(MESH_DIR, f'{mesh_name}.obj'), self.mesh_boxes)
            mesh = (f'<visual name="merged"><geometry><mesh><uri>{MESH_URI}/{mesh_name}.obj</uri>'
                    f'</mesh></geometry></visual>')
        return f'<link name="{self.name}">{inertial}{mesh}{"".join(self.items)}</link>'


def static_model(name, link, pose=(0, 0, 0)):
    return (f'<model name="{name}"><static>true</static><pose>{_pose(pose)}</pose>'
            f'{link.sdf(mesh_name=name)}</model>')


def _rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def write_obj(path, boxes):
    """Escribe cajas (pose, tamaño, color) como OBJ + MTL, agrupadas por material."""
    base = os.path.splitext(os.path.basename(path))[0]
    groups = {}
    for pose, size, color in boxes:
        key = color if isinstance(color, str) else 'c_' + '_'.join(f'{int(v * 255)}' for v in color)
        groups.setdefault(key, (color, []))[1].append((pose, size))
    corners = [(sx, sy, sz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    # caras (índices en corners) con su normal local, en sentido antihorario visto desde fuera
    faces = [((0, 1, 3, 2), (-1, 0, 0)), ((4, 6, 7, 5), (1, 0, 0)),
             ((0, 4, 5, 1), (0, -1, 0)), ((2, 3, 7, 6), (0, 1, 0)),
             ((0, 2, 6, 4), (0, 0, -1)), ((1, 5, 7, 3), (0, 0, 1))]
    obj = [f'mtllib {base}.mtl']
    mtl = []
    v_idx, n_idx = 1, 1
    for key, (color, items) in groups.items():
        c = C[color] if isinstance(color, str) else color
        mtl.append(f'newmtl {key}\nKa {c[0]} {c[1]} {c[2]}\nKd {c[0]} {c[1]} {c[2]}\n'
                   f'Ks 0.05 0.05 0.05\nNs 10\nd 1.0\nillum 2\n')
        obj.append(f'o {key}\nusemtl {key}')
        for pose, size in items:
            p = list(pose) + [0.0] * (6 - len(pose))
            R = _rot(p[3], p[4], p[5])
            for (a, b, cc) in corners:
                lx, ly, lz = a * size[0] / 2, b * size[1] / 2, cc * size[2] / 2
                obj.append('v %.4f %.4f %.4f' % tuple(
                    p[i] + R[i][0] * lx + R[i][1] * ly + R[i][2] * lz for i in range(3)))
            for (_, nrm) in faces:
                obj.append('vn %.4f %.4f %.4f' % tuple(
                    R[i][0] * nrm[0] + R[i][1] * nrm[1] + R[i][2] * nrm[2] for i in range(3)))
            for k, (idx, _) in enumerate(faces):
                vs = [v_idx + i for i in idx]
                n = n_idx + k
                obj.append(f'f {vs[0]}//{n} {vs[1]}//{n} {vs[2]}//{n}')
                obj.append(f'f {vs[0]}//{n} {vs[2]}//{n} {vs[3]}//{n}')
            v_idx += 8
            n_idx += 6
    with open(path, 'w') as f:
        f.write('\n'.join(obj) + '\n')
    with open(os.path.join(os.path.dirname(path), f'{base}.mtl'), 'w') as f:
        f.write('\n'.join(mtl))


def box_inertial(m, sx, sy, sz, z=0.0):
    return (f'<inertial><pose>0 0 {_f(z)} 0 0 0</pose><mass>{_f(m)}</mass><inertia>'
            f'<ixx>{_f(m * (sy * sy + sz * sz) / 12)}</ixx>'
            f'<iyy>{_f(m * (sx * sx + sz * sz) / 12)}</iyy>'
            f'<izz>{_f(m * (sx * sx + sy * sy) / 12)}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


# ============================================================================
# ELEMENTOS
# ============================================================================
def pallet_simple(link, x, y, z, cargo_h, cargo_color, along_x=False, collide=False):
    """Palet EUR (bloque) + carga. along_x: lado de 1.2 m paralelo a X."""
    px, py = (1.2, 0.8) if along_x else (0.8, 1.2)
    link.box((x, y, z + 0.072), (px, py, 0.144), 'wood', collide=False)
    # Huecos de entrada de horquillas (oscuros) en las dos caras visibles
    for s in (-1, 1):
        if along_x:
            link.box((x, y + s * 0.401, z + 0.05), (1.0, 0.002, 0.08), 'wood_dark', plot=False)
        else:
            link.box((x + s * 0.401, y, z + 0.05), (0.002, 1.0, 0.08), 'wood_dark', plot=False)
    if cargo_h > 0:
        cx, cy = px - 0.06, py - 0.06
        link.box((x, y, z + 0.144 + cargo_h / 2), (cx, cy, cargo_h), cargo_color)
    if collide:
        h = 0.144 + cargo_h
        link.box((x, y, z + h / 2), (px, py, h), 'wood', visual=False, collide=True)


def rack(link, rng, x0, y_min, n_bays):
    """Rack simple de n_bays vanos, fondo RACK_D, desde (x0, y_min)."""
    post = (0.09, 0.07)
    y_front, y_back = y_min + post[1] / 2, y_min + RACK_D - post[1] / 2
    depth_in = y_back - y_front
    yc = y_min + RACK_D / 2
    length = n_bays * BAY_W

    # Bastidores: 2 puestos + travesaños + diagonales
    for i in range(n_bays + 1):
        x = x0 + i * BAY_W
        for yp in (y_front, y_back):
            link.box((x, yp, UPRIGHT_H / 2), (post[0], post[1], UPRIGHT_H), 'upright')
        # Placa base
        for yp in (y_front, y_back):
            link.box((x, yp, 0.005), (0.15, 0.12, 0.01), 'steel', plot=False)
        seg = 1.2
        n_seg = int(UPRIGHT_H / seg)
        for k in range(n_seg + 1):
            z = 0.15 + k * seg if k < n_seg else UPRIGHT_H - 0.05
            link.box((x, yc, z), (0.04, depth_in, 0.04), 'upright', plot=False)
        for k in range(n_seg):
            z0 = 0.15 + k * seg
            dz = seg
            ang = math.atan2(dz, depth_in) * (1 if k % 2 == 0 else -1)
            L = math.hypot(dz, depth_in)
            link.box((x, yc, z0 + dz / 2, ang, 0, 0), (0.03, L, 0.03), 'upright', plot=False)

    # Largueros + palets
    slot_w = (BAY_W - post[0]) / SLOTS_PER_BAY
    for b in range(n_bays):
        xb = x0 + b * BAY_W + BAY_W / 2
        for zl in BEAM_LEVELS:
            for yp in (y_front, y_back):
                link.box((xb, yp, zl), (BAY_W - post[0], 0.05, BEAM_H), 'beam', plot=False)
        levels = [0.0] + [zl + BEAM_H / 2 for zl in BEAM_LEVELS]
        for li, zb in enumerate(levels):
            z_next = (BEAM_LEVELS[li] - BEAM_H / 2) if li < len(BEAM_LEVELS) else UPRIGHT_H
            max_h = min(1.15, z_next - zb - 0.144 - 0.12)
            for s in range(SLOTS_PER_BAY):
                if rng.random() > RACK_FILL:
                    continue
                xs = x0 + b * BAY_W + post[0] / 2 + slot_w * (s + 0.5)
                h = rng.uniform(0.6, max_h)
                pallet_simple(link, xs, yc, zb, h, rng.choice(CARGO_COLORS), along_x=False)

    # Protectores de extremo de fila (amarillos)
    for xe, sgn in ((x0, -1), (x0 + length, 1)):
        link.box((xe + sgn * 0.12, yc, 0.2), (0.08, RACK_D + 0.1, 0.4), 'yellow')
    # Colisión única del rack (huella completa: postes, palets que sobresalen y
    # protectores). Mantener pocas formas de colisión es clave para el RTF.
    link.box((x0 + length / 2, yc, UPRIGHT_H / 2), (length + 0.32, RACK_D + 0.1, UPRIGHT_H),
             'upright', visual=False, collide=True)
    return length


def rack_rows(rng):
    out = []
    for name, y_min, n in RACK_ROWS:
        link = Link()
        y = y_min
        for k in range(n):
            rack(link, rng, RACK_X0, y, N_BAYS)
            y += RACK_D + FLUE
        out.append(static_model(name, link))
    return out


def building():
    link = Link()
    x0, x1 = HALL_X
    y0, y1 = HALL_Y
    W, H = x1 - x0, y1 - y0
    t = WALL_T
    # Muros
    for (x, y, sx, sy) in [((x0 + x1) / 2, y1 + t / 2, W + 2 * t, t),
                           ((x0 + x1) / 2, y0 - t / 2, W + 2 * t, t),
                           (x1 + t / 2, (y0 + y1) / 2, t, H),
                           (x0 - t / 2, (y0 + y1) / 2, t, H)]:
        link.box((x, y, WALL_H / 2), (sx, sy, WALL_H), 'wall', collide=True)
    # Zócalo pintado (banda azul de 1 m) en las caras interiores
    band = 1.0
    link.box(((x0 + x1) / 2, y1 - 0.005, band / 2), (W, 0.01, band), 'wall_band', plot=False)
    link.box(((x0 + x1) / 2, y0 + 0.005, band / 2), (W, 0.01, band), 'wall_band', plot=False)
    link.box((x1 - 0.005, (y0 + y1) / 2, band / 2), (0.01, H, band), 'wall_band', plot=False)
    link.box((x0 + 0.005, (y0 + y1) / 2, band / 2), (0.01, H, band), 'wall_band', plot=False)

    # Puertas de muelle (enrollables, cerradas) + topes de goma + nivelador
    for xd in DOCK_DOORS_X:
        link.box((xd, y0 + 0.02, DOCK_DOOR_H / 2), (DOCK_DOOR_W, 0.04, DOCK_DOOR_H), 'door', plot=False)
        for k in range(8):   # lamas
            link.box((xd, y0 + 0.045, 0.2 + k * 0.45), (DOCK_DOOR_W - 0.05, 0.01, 0.03), 'steel', plot=False)
        for s in (-1, 1):    # marco amarillo
            link.box((xd + s * (DOCK_DOOR_W / 2 + 0.08), y0 + 0.06, DOCK_DOOR_H / 2),
                     (0.16, 0.12, DOCK_DOOR_H), 'yellow', collide=True)
            link.box((xd + s * (DOCK_DOOR_W / 2 - 0.3), y0 + 0.08, 0.25),
                     (0.25, 0.16, 0.35), 'black', collide=True)
        link.box((xd, y0 + 0.5, 0.004), (2.2, 1.0, 0.008), 'steel')    # nivelador de muelle
        # Señal luminosa del muelle
        link.box((xd + DOCK_DOOR_W / 2 + 0.4, y0 + 0.05, 2.0), (0.15, 0.05, 0.3), 'black', plot=False)
        link.box((xd + DOCK_DOOR_W / 2 + 0.4, y0 + 0.08, 2.07), (0.08, 0.02, 0.08), 'green',
                 emissive=(0.1, 0.8, 0.2), plot=False)

    # Puerta peatonal (muro oeste, cerrada) y extintores
    link.box((x0 + 0.03, -8.0, 1.05), (0.04, 1.0, 2.1), 'door', plot=False)
    for (x, y) in [(x0 + 0.1, -3.0), (x0 + 0.1, 8.0), (x1 - 0.1, -3.0), (0.0, y1 - 0.1), (-10.0, y0 + 0.1)]:
        link.box((x, y, 0.75), (0.15, 0.15, 0.5), 'red', plot=False)
    return static_model('building', link)


def floor():
    link = Link()
    link.items.append(
        '<collision name="ground"><geometry><plane><normal>0 0 1</normal><size>80 60</size></plane></geometry>'
        '<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>'
        f'<visual name="ground_v"><geometry><plane><normal>0 0 1</normal><size>80 60</size></plane></geometry>'
        f'{_material((0.30, 0.31, 0.30))}</visual>')
    x0, x1 = HALL_X
    y0, y1 = HALL_Y
    # Losa de hormigón interior
    link.box(((x0 + x1) / 2, (y0 + y1) / 2, 0.0005), (x1 - x0, y1 - y0, 0.001), 'floor')
    return static_model('floor', link)


def floor_markings():
    link = Link()
    x_end = RACK_X0 + N_BAYS * BAY_W
    lw = 0.1
    # Líneas amarillas delimitando pasillos de racks + eje discontinuo blanco
    for (ya, yb) in AISLES_Y:
        for yl in (ya + 0.15, yb - 0.15):
            link.marking((RACK_X0 + x_end) / 2, yl, x_end - RACK_X0, lw, 'yellow')
        yc = (ya + yb) / 2
        x = RACK_X0 + 0.5
        while x + 1.0 < x_end:
            link.marking(x + 0.5, yc, 1.0, 0.08, 'white')
            x += 2.0
    # Límite de la zona de preparación
    link.marking((RACK_X0 + 15.0) / 2, STAGING_Y[1], 15.0 - RACK_X0, lw, 'yellow')
    # Pasarela peatonal verde con bordes blancos y paso de cebra
    wx0, wx1 = WALKWAY_X
    yw0, yw1 = STAGING_Y[1] + 0.3, HALL_Y[1] - 0.3
    link.marking((wx0 + wx1) / 2, (yw0 + yw1) / 2, wx1 - wx0, yw1 - yw0, 'green')
    for xe in (wx0, wx1):
        link.marking(xe, (yw0 + yw1) / 2, 0.06, yw1 - yw0, 'white')
    # Ramal peatonal hasta la puerta de la oficina
    ox0, ox1 = OFFICE_DOOR_X
    link.marking((wx1 + ox1) / 2, 5.0, ox1 - wx1, 0.8, 'green')
    link.marking((ox0 + ox1) / 2, (5.4 + OFFICE[2]) / 2, ox1 - ox0, OFFICE[2] - 5.4, 'green')
    for k in range(5):   # paso de cebra en el cruce con el pasillo transversal
        link.marking((wx0 + wx1) / 2, STAGING_Y[1] + 0.6 + k * 0.5, wx1 - wx0, 0.25, 'white')
    # Estaciones de palet: recuadro del color del área y flecha de aproximación
    for st in STATIONS:
        col = AREA_COLORS[st['area']]
        for (lx, ly, sx, sy) in [(0, 0.95, 1.8, lw), (0, -0.95, 1.8, lw), (0.85, 0, lw, 1.9), (-0.85, 0, lw, 1.9)]:
            wx, wy = station_to_world(st, lx, ly)
            link.marking(wx, wy, sx, sy, col, yaw=st['yaw'])
        for (lx, ly, sx, sy, a) in [(0, 1.6, 0.1, 0.9, 0), (-0.12, 1.25, 0.32, 0.08, 50), (0.12, 1.25, 0.32, 0.08, -50)]:
            wx, wy = station_to_world(st, lx, ly)
            link.marking(wx, wy, sx, sy, 'white', yaw=st['yaw'] + math.radians(a))
    # Contorno de cada área de destino
    for area in ('area_1', 'area_2', 'area_3'):
        pts = [station_to_world(st, lx, ly) for st in STATIONS if st['area'] == area
               for lx in (-1.0, 1.0) for ly in (-1.1, 1.1)]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        link.outline((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
                     max(xs) - min(xs) + 0.3, max(ys) - min(ys) + 0.3, AREA_COLORS[area], lw=0.15)
    # Zona libre frente a cada muelle (rayado amarillo)
    for xd in DOCK_DOORS_X:
        for k in range(5):
            link.marking(xd - 1.2 + k * 0.6, HALL_Y[0] + 1.35, 0.12, 0.5, 'yellow', yaw=math.radians(35))
    # Estación de carga del AMR
    for yc in CHARGERS_Y:
        link.outline(HALL_X[1] - 1.0, yc, 1.4, 1.0, 'green')
    # Plaza de carretilla
    fx, fy, _ = FORKLIFT
    link.outline(fx + 0.3, fy, 3.6, 2.2, 'white')
    # Almacenaje en bloque: recuadro
    link.outline(6.6, 10.4, 7.6, 2.9, 'yellow')
    return static_model('floor_markings', link)


def columns():
    link = Link()
    for (x, y) in COLUMNS:
        link.box((x, y, WALL_H / 2), (0.5, 0.5, WALL_H), 'concrete')
        link.box((x, y, 0.4), (0.62, 0.62, 0.8), 'yellow')
        link.box((x, y, WALL_H / 2), (0.62, 0.62, WALL_H), 'concrete', visual=False, collide=True)
        for k in range(3):
            link.box((x, y, 0.15 + k * 0.25), (0.63, 0.63, 0.08), 'black', plot=False)
    return static_model('columns', link)


def office():
    link = Link()
    x0, x1, y0, y1 = OFFICE
    h, t = 3.0, 0.15
    dx0, dx1 = OFFICE_DOOR_X

    def wall(xa, xb, ya, yb):
        cx, cy = (xa + xb) / 2, (ya + yb) / 2
        sx, sy = max(abs(xb - xa), t), max(abs(yb - ya), t)
        link.box((cx, cy, 0.5), (sx, sy, 1.0), 'wall', collide=True)
        link.box((cx, cy, 1.6), (sx * 0.999 if sx > t else t * 0.6, sy * 0.999 if sy > t else t * 0.6, 1.2),
                 'glass', alpha=0.45, collide=True, plot=False)
        link.box((cx, cy, 2.6), (sx, sy, 0.8), 'wall', plot=False)

    wall(x0, x0, y0, y1)            # oeste
    wall(x0, dx0, y0, y0)           # sur (izq. de la puerta)
    wall(dx1, x1, y0, y0)           # sur (dcha. de la puerta)
    link.box(((dx0 + dx1) / 2, y0, 2.6), (dx1 - dx0, t, 0.8), 'wall', plot=False)   # dintel
    link.box((x0 + (x1 - x0) / 2, (y0 + y1) / 2, h + 0.05), (x1 - x0, y1 - y0, 0.1), 'steel', plot=False)
    # Mobiliario
    for (x, y) in [(16.5, 9.5), (18.5, 9.5), (18.5, 7.8)]:
        link.box((x, y, 0.75), (1.4, 0.7, 0.04), 'wood')
        for sx in (-0.6, 0.6):
            link.box((x + sx, y, 0.37), (0.05, 0.6, 0.74), 'steel', plot=False)
        link.box((x, y + 0.6, 0.45), (0.5, 0.5, 0.1), 'navy')
    link.box((19.6, 11.2, 1.0), (0.6, 1.2, 2.0), 'crate_grey')   # armario
    return static_model('office', link)


def block_storage(rng):
    """Palets apilados directamente en el suelo (1-2 alturas)."""
    link = Link()
    x = 3.1
    for col in range(8):
        for row_y in (9.55, 11.25):
            if rng.random() < 0.15:
                continue
            h1 = rng.uniform(0.7, 1.0)
            pallet_simple(link, x, row_y, 0.0, h1, rng.choice(CARGO_COLORS), along_x=False)
            if rng.random() < 0.55:
                pallet_simple(link, x, row_y, 0.144 + h1, rng.uniform(0.5, 0.9),
                              rng.choice(CARGO_COLORS), along_x=False)
        x += 0.95
    for row_y in (9.55, 11.25):   # una colisión por fila
        link.box(((3.1 + x - 0.95) / 2, row_y, 1.0), (x - 0.95 - 3.1 + 0.8, 1.2, 2.0), 'wood',
                 visual=False, collide=True)
    # Pila de palets vacíos
    for k in range(9):
        pallet_simple(link, 11.2, 10.4, k * 0.144, 0.0, 'wood', along_x=True)
    link.box((11.2, 10.4, 9 * 0.144 / 2), (1.2, 0.8, 9 * 0.144), 'wood', visual=False, collide=True)
    return static_model('block_storage', link)


def chargers():
    link = Link()
    for yc in CHARGERS_Y:
        xw = HALL_X[1]
        link.box((xw - 0.15, yc, 0.4), (0.3, 0.7, 0.8), 'navy', collide=True)
        link.box((xw - 0.31, yc, 0.6), (0.02, 0.3, 0.12), 'black', plot=False)
        link.box((xw - 0.32, yc + 0.1, 0.72), (0.01, 0.05, 0.05), 'green', emissive=(0.1, 0.9, 0.2), plot=False)
        for s in (-1, 1):   # contactos de carga a la altura del AMR
            link.box((xw - 0.33, yc + s * 0.15, 0.2), (0.04, 0.08, 0.05), (0.72, 0.45, 0.20), plot=False)
    return static_model('amr_chargers', link)


def conveyor(rng):
    link = Link()
    xc, ya, yb = CONVEYOR
    L, w, h = yb - ya, 0.8, 0.75
    yc = (ya + yb) / 2
    link.box((xc, yc, h - 0.03), (w, L, 0.06), 'crate_grey')
    link.box((xc, (ya - 0.1 + yb) / 2, (h + 0.25) / 2), (w + 0.1, L + 0.1, h + 0.25), 'steel',
             visual=False, collide=True)
    for s in (-1, 1):
        link.box((xc + s * (w / 2 + 0.02), yc, h + 0.02), (0.04, L, 0.12), 'steel', plot=False)
    n_legs = int(L / 1.5) + 1
    for k in range(n_legs):
        y = ya + 0.1 + k * (L - 0.2) / (n_legs - 1)
        for s in (-1, 1):
            link.box((xc + s * (w / 2 - 0.05), y, (h - 0.06) / 2), (0.06, 0.06, h - 0.06), 'steel')
        link.box((xc, y, 0.15), (w - 0.1, 0.04, 0.04), 'steel', plot=False)
    for k in range(int(L / 0.12)):
        link.box((xc, ya + 0.06 + k * 0.12, h + 0.005), (w - 0.04, 0.04, 0.01), 'black', plot=False)
    for y in (ya + 1.0, ya + 2.6, ya + 4.5):
        s = rng.uniform(0.35, 0.55)
        link.box((xc, y, h + s / 2 + 0.01), (s, s, s), 'cardboard')
    # Final de línea + armario eléctrico
    link.box((xc, ya - 0.05, h + 0.1), (w + 0.1, 0.1, 0.3), 'yellow')
    link.box((xc - 0.9, yb - 0.3, 0.9), (0.5, 0.4, 1.8), 'crate_grey', collide=True)
    link.box((xc - 0.66, yb - 0.3, 1.5), (0.02, 0.1, 0.1), 'red', emissive=(0.8, 0.1, 0.1), plot=False)
    return static_model('conveyor', link)


def forklift():
    x, y, yaw = FORKLIFT
    link = Link(origin=(x, y, 0.0, yaw))
    link.box((0.0, 0, 0.55), (1.6, 1.05, 0.7), 'orange')                          # cuerpo
    link.box((-0.75, 0, 0.6), (0.35, 1.0, 0.8), 'crate_grey')                     # contrapeso
    link.box((-0.06, 0, 1.1), (1.75, 1.15, 2.2), 'orange', visual=False, collide=True)
    link.box((0.05, 0, 0.95), (0.5, 0.6, 0.1), 'black')                            # asiento
    for (px, py) in [(-0.45, 0.48), (-0.45, -0.48), (0.55, 0.48), (0.55, -0.48)]:  # techo protector
        link.box((px, py, 1.55), (0.05, 0.05, 1.2), 'black', plot=False)
    link.box((0.05, 0, 2.15), (1.05, 1.0, 0.04), 'black', plot=False)
    for s in (-1, 1):                                                              # mástil
        link.box((0.9, s * 0.3, 1.2), (0.08, 0.1, 2.3), 'crate_grey')
    link.box((0.97, 0, 0.35), (0.06, 0.8, 0.5), 'black', plot=False)
    for s in (-1, 1):                                                              # horquillas
        link.box((1.55, s * 0.25, 0.04), (1.15, 0.12, 0.05), 'crate_grey')
    link.box((1.55, 0, 0.04), (1.15, 0.62, 0.05), 'crate_grey', visual=False, collide=True)
    for (px, py, r) in [(0.5, 0.45, 0.25), (0.5, -0.45, 0.25), (-0.55, 0.45, 0.2), (-0.55, -0.45, 0.2)]:
        link.cylinder((px, py, r, math.pi / 2, 0, 0), r, 0.2, 'black')
    return static_model('forklift', link, pose=(x, y, 0, 0, 0, yaw))


def person(name, x, y, yaw):
    link = Link(origin=(x, y, 0.0, yaw))
    for s in (-1, 1):
        link.cylinder((0, s * 0.1, 0.43), 0.07, 0.86, 'navy', collide=True)       # piernas
        link.box((0.05, s * 0.1, 0.04), (0.26, 0.11, 0.08), 'black', plot=False)  # botas
        link.cylinder((0, s * 0.27, 1.12), 0.05, 0.6, 'hivis')                    # brazos
    link.box((0, 0, 1.17), (0.25, 0.42, 0.62), 'hivis', collide=True)             # torso (chaleco)
    link.cylinder((0, 0, 1.54), 0.05, 0.1, 'skin')
    link.sphere((0, 0, 1.68), 0.11, 'skin')
    link.box((0, 0, 1.79), (0.26, 0.26, 0.06), 'yellow', plot=False)              # casco
    return static_model(name, link, pose=(x, y, 0, 0, 0, yaw))


def pallet_station(st):
    """Estación elevada: el palet se apoya por sus extremos cortos.
    Hueco libre 1.00 m (x local) x 0.41 m (z); el AMR entra por +y local."""
    x, y, yaw = st['x'], st['y'], st['yaw']
    link = Link(origin=(x, y, 0.0, yaw))
    for s in (-1, 1):
        link.box((s * 0.60, 0, 0.43), (0.20, 1.0, 0.04), 'orange', collide=True)
        for t in (-1, 1):
            link.box((s * 0.65, t * 0.45, 0.205), (0.08, 0.08, 0.41), 'orange', collide=True)
            link.box((s * 0.65, t * 0.45, 0.005), (0.16, 0.16, 0.01), 'steel', plot=False)
        link.box((s * 0.60, 0, 0.47), (0.04, 0.9, 0.04), 'yellow', plot=False)   # topes laterales
    return static_model(f'station_{st["id"].replace("-", "_")}', link, pose=(x, y, 0, 0, 0, yaw))


def euro_pallet_dynamic(name, x, y, z, yaw, cargo_kg, rng):
    """Palet EUR detallado y dinámico (con carga flejada) para el AMR."""
    p = Link('pallet', origin=(x, y, z, yaw), merge=False)
    # Tablas superiores (5) e inferiores (3), tacos (9), largueros (3)
    for k, yb in enumerate([-0.3525, -0.17625, 0.0, 0.17625, 0.3525]):
        w = 0.145 if k in (0, 2, 4) else 0.1
        p.box((0, yb, 0.133), (1.2, w, 0.022), 'wood')
    for yb in (-0.3525, 0.0, 0.3525):
        p.box((0, yb, 0.111), (1.2, 0.145, 0.022), 'wood_dark', plot=False)
        p.box((0, yb, 0.011), (1.2, 0.145, 0.022), 'wood_dark', plot=False)
        for xb in (-0.5275, 0.0, 0.5275):
            p.box((xb, yb, 0.061), (0.145, 0.145, 0.078), 'wood', plot=False)
    p.box((0, 0, 0.072), (1.2, 0.8, 0.144), 'wood', visual=False, collide=True)
    mass_p = 25.0
    pallet = (f'<link name="pallet"><pose>0 0 0 0 0 0</pose>{box_inertial(mass_p, 1.2, 0.8, 0.144, 0.072)}'
              f'{"".join(p.items)}</link>')
    parts = [pallet]
    if cargo_kg:
        h = 0.9
        c = Link('cargo', origin=(x, y, z, yaw), merge=False)
        c.box((0, 0, 0.144 + h / 2), (1.14, 0.76, h), rng.choice(['wrap', 'cardboard']), collide=True)
        for xs in (-0.3, 0.3):   # flejes
            c.box((xs, 0, 0.144 + h / 2), (0.03, 0.765, h + 0.005), 'black', plot=False)
        cargo = (f'<link name="cargo"><pose>0 0 0 0 0 0</pose>'
                 f'{box_inertial(cargo_kg, 1.14, 0.76, h, 0.144 + h / 2)}{"".join(c.items)}</link>')
        parts.append(cargo)
        parts.append('<joint name="strapped" type="fixed"><parent>pallet</parent><child>cargo</child></joint>')
    return f'<model name="{name}"><pose>{_pose((x, y, z, 0, 0, yaw))}</pose>{"".join(parts)}</model>'


def staging_floor_pallets(rng):
    """Palets en espera junto a los muelles (estáticos)."""
    link = Link()
    for xd in DOCK_DOORS_X[:-1]:
        x = xd + 2.5
        for k, y in enumerate((-11.0, -9.9)):
            pallet_simple(link, x, y, 0.0, rng.uniform(0.6, 1.3), rng.choice(CARGO_COLORS),
                          along_x=False, collide=True)
    return static_model('staging_pallets', link)


def lights():
    out = ['''<light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 30 0 0 0</pose>
      <diffuse>0.45 0.45 0.45 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <direction>-0.3 0.2 -0.9</direction></light>''']
    k = 0
    for x in (-14.0, -6.0, 2.0, 10.0, 17.0):
        for y in (-6.0, 5.0):
            k += 1
            out.append(f'''<light type="point" name="ceiling_{k}">
      <pose>{x} {y} 7.5 0 0 0</pose><cast_shadows>false</cast_shadows>
      <diffuse>0.22 0.22 0.20 1</diffuse><specular>0.0 0.0 0.0 1</specular>
      <attenuation><range>22</range><constant>0.4</constant><linear>0.03</linear><quadratic>0.002</quadratic></attenuation>
    </light>''')
    return out


# ============================================================================
# MUNDO
# ============================================================================
def build_world():
    rng = random.Random(SEED)
    models = [floor(), building(), floor_markings(), columns(), office()]
    models += rack_rows(rng)
    models += [block_storage(rng), chargers(), conveyor(rng), forklift(), staging_floor_pallets(rng)]
    for st in STATIONS:
        models.append(pallet_station(st))
        if st['cargo_kg']:
            models.append(euro_pallet_dynamic(f'pallet_{st["id"].replace("-", "_")}', st['x'], st['y'],
                                              0.45, st['yaw'], st['cargo_kg'], rng))
    models += [person(*p) for p in PEOPLE]

    header = f'''<?xml version="1.0"?>
<!--
  ARCHIVO GENERADO por scripts/generate_warehouse.py — no editar a mano.
  Depósito industrial de {HALL_X[1] - HALL_X[0]:.0f} x {HALL_Y[1] - HALL_Y[0]:.0f} m:
    - {sum(n for _, _, n in RACK_ROWS)} racks de palets ({N_BAYS} vanos x {len(BEAM_LEVELS) + 1} niveles) y pasillos de 3.4 m
    - zona de carga con {len(DOCK_DOORS_X)} muelles y estaciones P1..P4 (palets dinámicos pallet_Pn)
    - áreas de destino area_1..3 con estaciones vacías (ver config/warehouse_stations.yaml)
    - almacenaje en bloque, columnas, oficina, cinta transportadora, carretilla,
      cargadores del AMR y personas estáticas (person_N) para pruebas de seguridad
  Origen (0,0): pasillo transversal libre, punto de spawn por defecto del AMR.
-->
<sdf version="1.10">
  <world name="warehouse">
    <physics name="2ms" type="ode">
      <!-- 2 ms: la mitad de mensajes /clock para todos los nodos ROS y menos
           coste de física; estable con las cargas de 1000 kg (validado) -->
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.40 0.40 0.40 1</ambient>
      <background>0.70 0.78 0.85 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
'''
    body = '\n    '.join(lights() + models)
    return header + '    ' + body + '\n  </world>\n</sdf>\n'


def plot(path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    import matplotlib.transforms as mt

    fig, ax = plt.subplots(figsize=(16, 10))
    for (x, y, sx, sy, yaw, color, ztop) in sorted(PLOT_ITEMS, key=lambda it: it[6]):
        c = C[color] if isinstance(color, str) else color
        r = Rectangle((x - sx / 2, y - sy / 2), sx, sy, facecolor=c, edgecolor='none')
        r.set_transform(mt.Affine2D().rotate_around(x, y, yaw) + ax.transData)
        ax.add_patch(r)
    for st in STATIONS:
        lx, ly = station_to_world(st, 0.0, -1.35)
        label = st['id'] + (f'\n{int(st["cargo_kg"])} kg' if st['cargo_kg'] else '')
        ax.text(lx, ly, label, ha='center', va='center', fontsize=8, fontweight='bold')
        (px, py, pyaw), _ = station_poses(st)
        ax.annotate('', xy=(st['x'], st['y']), xytext=(px, py),
                    arrowprops=dict(arrowstyle='->', color='black', lw=0.8))
    for area, col in AREA_COLORS.items():
        sts = [st for st in STATIONS if st['area'] == area]
        cx = sum(st['x'] for st in sts) / len(sts)
        cy = max(st['y'] for st in sts) + 1.9
        ax.text(cx, cy, area.upper().replace('_', ' '), ha='center', fontsize=10, fontweight='bold',
                color=col if isinstance(col, tuple) else 'darkgoldenrod')
    for name, x, y, _ in PEOPLE:
        ax.text(x + 0.3, y + 0.3, name, fontsize=7, color='darkred')
    ax.plot(0, 0, marker='*', color='red', markersize=14)
    ax.text(0.3, 0.3, 'spawn (0,0)', color='red', fontsize=8)
    ax.set_xlim(HALL_X[0] - 1, HALL_X[1] + 1)
    ax.set_ylim(HALL_Y[0] - 1, HALL_Y[1] + 1)
    ax.set_aspect('equal')
    ax.set_xticks(range(int(HALL_X[0]), int(HALL_X[1]) + 1, 2))
    ax.set_yticks(range(int(HALL_Y[0]), int(HALL_Y[1]) + 1, 2))
    ax.grid(alpha=0.25)
    ax.set_title('Depósito AMR — plano (vista superior)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def keepout_zones():
    """Zonas por las que el AMR no debe circular (paredes virtuales de navegación).
    Los LiDAR solo ven postes y palets a ras de suelo: sin estas zonas, un vano
    vacío de un rack parece un paso libre."""
    m = 0.05
    x_end = RACK_X0 + N_BAYS * BAY_W
    zones = []
    for name, y_min, n in RACK_ROWS:
        depth = n * RACK_D + (n - 1) * FLUE
        zones.append((name, RACK_X0 - 0.2, x_end + 0.2, y_min - 0.05 - m, y_min + depth + 0.05 + m))
    zones.append(('block_storage', 2.6, 10.2, 8.9, HALL_Y[1]))
    zones.append(('office', OFFICE[0] - m, OFFICE[1], OFFICE[2] - m, OFFICE[3]))
    xc, ya, yb = CONVEYOR
    zones.append(('conveyor', xc - 1.2, HALL_X[1], ya - 0.15, yb + 0.1))
    return zones


def write_keepout_yaml(path):
    lines = ['# ARCHIVO GENERADO por scripts/generate_warehouse.py — no editar a mano.',
             '# Zonas prohibidas para la navegación [x_min, x_max, y_min, y_max] (frame map).',
             '# Las usa amr_navigation/scripts/make_keepout_map.py.',
             'zones:']
    for (name, x0, x1, y0, y1) in keepout_zones():
        lines.append(f'  - {{name: {name}, box: [{x0:.2f}, {x1:.2f}, {y0:.2f}, {y1:.2f}]}}')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'zonas prohibidas: {os.path.realpath(path)}')


def write_stations_yaml(path):
    """Exporta las estaciones (fuente única para el gestor de tareas)."""
    lines = ['# ARCHIVO GENERADO por scripts/generate_warehouse.py — no editar a mano.',
             '# Poses en el frame "map" (origen del mapa = origen del mundo de Gazebo).',
             '#   pre_dock: pose del robot frente a la estación (x, y, yaw)',
             '#   dock:     pose del robot centrado bajo la estación',
             '#   loaded:   la estación tiene un palet al inicio',
             'stations:']
    for st in STATIONS:
        pre, dock = station_poses(st)
        lines += [f'  - id: "{st["id"]}"',
                  f'    area: "{st["area"]}"',
                  f'    pre_dock: [{pre[0]:.3f}, {pre[1]:.3f}, {pre[2]:.4f}]',
                  f'    dock: [{dock[0]:.3f}, {dock[1]:.3f}, {dock[2]:.4f}]',
                  f'    loaded: {"true" if st["cargo_kg"] else "false"}']
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f'estaciones: {os.path.realpath(path)}')


def main():
    here = os.path.dirname(os.path.realpath(__file__))
    default_out = os.path.join(here, '..', 'worlds', 'warehouse.sdf')
    ap = argparse.ArgumentParser(description='Genera el mundo warehouse.sdf')
    ap.add_argument('-o', '--output', default=default_out)
    ap.add_argument('--plot', help='ruta PNG para el plano del depósito')
    args = ap.parse_args()

    global MESH_DIR
    MESH_DIR = os.path.join(os.path.dirname(os.path.realpath(args.output)), 'meshes')
    os.makedirs(MESH_DIR, exist_ok=True)
    for f in os.listdir(MESH_DIR):
        if f.endswith(('.obj', '.mtl')):
            os.remove(os.path.join(MESH_DIR, f))

    sdf = build_world()
    with open(args.output, 'w') as f:
        f.write(sdf)
    n_vis = sdf.count('<visual ')
    n_col = sdf.count('<collision ')
    n_box = sum(len(open(os.path.join(MESH_DIR, f)).read().split('\no ')) - 1
                for f in os.listdir(MESH_DIR) if f.endswith('.obj'))
    print(f'{os.path.realpath(args.output)}: {len(sdf) / 1e6:.2f} MB, {n_vis} visuales SDF '
          f'({n_box} submallas fusionadas en {MESH_DIR}), {n_col} colisiones')
    write_stations_yaml(os.path.join(here, '..', 'config', 'warehouse_stations.yaml'))
    write_keepout_yaml(os.path.join(here, '..', 'config', 'warehouse_keepout.yaml'))
    if args.plot:
        plot(args.plot)
        print(f'plano: {args.plot}')


if __name__ == '__main__':
    main()
