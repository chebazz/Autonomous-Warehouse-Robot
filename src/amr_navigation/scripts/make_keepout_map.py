#!/usr/bin/env python3
"""Genera el mapa de navegación con zonas prohibidas (paredes virtuales).

    make_keepout_map.py <mapa.yaml> <zonas.yaml> [-o salida_sin_extension]

Rellena como ocupadas las zonas de <zonas.yaml> (racks, almacenaje en bloque,
oficina...) sobre el mapa SLAM y guarda <mapa>_keepout.pgm/.yaml.

  - AMCL usa el mapa SLAM original (lo que realmente ven los LiDAR).
  - Los costmaps de Nav2 usan el mapa keepout: el planificador nunca traza
    rutas por un vano vacío de un rack ni por zonas no transitables.
"""
import argparse
import os

import yaml


def read_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'{path}: se esperaba PGM binario (P5)')
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline())
        data = bytearray(f.read(w * h))
    return w, h, maxval, data


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('zones_yaml')
    ap.add_argument('-o', '--output', help='ruta de salida sin extensión (por defecto <mapa>_keepout)')
    args = ap.parse_args()

    with open(args.map_yaml) as f:
        meta = yaml.safe_load(f)
    base_dir = os.path.dirname(os.path.realpath(args.map_yaml))
    w, h, maxval, data = read_pgm(os.path.join(base_dir, meta['image']))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    occupied = 0 if not meta.get('negate', 0) else 255

    with open(args.zones_yaml) as f:
        zones = yaml.safe_load(f)['zones']
    for z in zones:
        x0, x1, y0, y1 = z['box']
        c0, c1 = max(0, int((x0 - ox) / res)), min(w - 1, int((x1 - ox) / res))
        r0, r1 = max(0, int((y0 - oy) / res)), min(h - 1, int((y1 - oy) / res))
        for r in range(r0, r1 + 1):
            row = h - 1 - r                       # el PGM tiene la fila 0 arriba
            data[row * w + c0: row * w + c1 + 1] = bytes([occupied]) * (c1 - c0 + 1)
        print(f'  zona {z["name"]:14s} x[{x0:.2f}, {x1:.2f}] y[{y0:.2f}, {y1:.2f}]')

    out = args.output or os.path.splitext(os.path.realpath(args.map_yaml))[0] + '_keepout'
    with open(out + '.pgm', 'wb') as f:
        f.write(b'P5\n# zonas prohibidas aplicadas\n%d %d\n%d\n' % (w, h, maxval))
        f.write(bytes(data))
    meta['image'] = os.path.basename(out) + '.pgm'
    with open(out + '.yaml', 'w') as f:
        yaml.safe_dump(meta, f, default_flow_style=None, sort_keys=False)
    print(f'Mapa de navegación: {out}.yaml ({len(zones)} zonas)')


if __name__ == '__main__':
    main()
