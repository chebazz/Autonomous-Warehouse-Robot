#!/bin/bash
# Guarda el mapa que está construyendo slam_toolbox.
#   ros2 run amr_navigation save_map.sh [nombre] [directorio]
# Por defecto: warehouse en src/amr_navigation/maps (si existe) o en el directorio actual.
#   <nombre>.pgm/.yaml  → map_server (navegación con AMCL)
#   <nombre>.posegraph/.data → grafo de slam_toolbox (continuar mapeo / localización)
NAME=${1:-warehouse}
DIR=${2:-}
if [ -z "$DIR" ]; then
  SRC=$(ros2 pkg prefix amr_navigation)/../../src/amr_navigation/maps
  if [ -d "$SRC" ]; then DIR=$(realpath "$SRC"); else DIR=$(pwd); fi
fi
mkdir -p "$DIR"
echo "Guardando mapa en $DIR/$NAME ..."
ros2 run nav2_map_server map_saver_cli -f "$DIR/$NAME" --ros-args -p use_sim_time:=true \
  -p save_map_timeout:=10.0 -p free_thresh_default:=0.196 -p occupied_thresh_default:=0.65
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$DIR/$NAME'}" >/dev/null && echo "Grafo SLAM guardado: $DIR/$NAME.posegraph"
# Mapa de navegación con zonas prohibidas (racks, oficina...) para los costmaps
ZONES=$(ros2 pkg prefix amr_gazebo)/share/amr_gazebo/config/warehouse_keepout.yaml
[ -f "$ZONES" ] && ros2 run amr_navigation make_keepout_map.py "$DIR/$NAME.yaml" "$ZONES"
