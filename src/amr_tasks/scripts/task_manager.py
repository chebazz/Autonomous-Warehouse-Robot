#!/usr/bin/env python3
"""Gestor de tareas de transporte de palets del AMR.

Cada TAREA es un viaje: recoger un palet en el ÁREA DE CARGA y llevarlo a una
estación libre de su ÁREA DESIGNADA. Las tareas se ejecutan en orden; al
terminar la cola el AMR vuelve a la base.

Secuencia de un viaje
    1. Nav2 → pose de pre-atraque de la estación de recogida
    2. alineación fina y entrada recta bajo la estación (control holonómico)
    3. elevar a la altura de transporte (separa el palet de las ménsulas)
    4. huella "cargado" en los costmaps → salida recta hasta el pre-atraque
    5. Nav2 → pre-atraque de la estación destino → entrada → bajar → salida
    6. huella "vacío", actualizar ocupación de estaciones

Entradas
    ~stations_file  YAML de estaciones (generado por amr_gazebo/generate_warehouse.py)
    ~tasks_file     YAML de tareas (ver amr_tasks/config/tasks.yaml)
    /amr/task_request (std_msgs/String)  añadir tareas en caliente:
        "area_2"            → palet automático del área de carga → area_2
        "P3>area_1"         → palet de la estación P3 → area_1
        "3x area_1,area_2"  → 3 viajes alternando area_1, area_2
        "reanudar"          → salir de la pausa tras un fallo con carga
Salidas
    /amr/task_status (std_msgs/String, JSON)   estado y progreso
    /amr/stations_markers (MarkerArray)        estaciones en RViz (ocupación)
"""
import json
import math
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point32, Polygon, PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

PICKUP_AREA = 'carga'
FOOTPRINT_EMPTY = [(0.62, 0.42), (0.62, -0.42), (-0.62, -0.42), (-0.62, 0.42)]
FOOTPRINT_LOADED = [(0.65, 0.65), (0.65, -0.65), (-0.65, -0.65), (-0.65, 0.65)]
AREA_RGB = {'carga': (0.95, 0.78, 0.05), 'area_1': (0.10, 0.40, 0.90),
            'area_2': (0.60, 0.20, 0.80), 'area_3': (0.00, 0.70, 0.75)}


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class TaskError(Exception):
    pass


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        p = self.declare_parameter
        p('stations_file', '')
        p('tasks_file', '')
        p('lift_transport_height', 0.10)
        p('lift_speed', 0.05)
        p('dock_speed', 0.15)
        p('dock_xy_tolerance', 0.02)
        p('dock_yaw_tolerance', 0.02)
        p('nav_retries', 2)
        p('autostart', True)
        gp = lambda n: self.get_parameter(n).value

        with open(gp('stations_file')) as f:
            self.stations = {s['id']: s for s in yaml.safe_load(f)['stations']}
        self.reserved = set()
        self.home = [0.0, 0.0, 0.0]
        self.queue = []
        self.history = []
        self.lock = threading.Lock()
        self.state = 'iniciando'
        self.current = None
        self.loaded = False          # palet elevado sobre el robot
        self.docked_at = None        # estación bajo la que está el robot
        self.blocked = set()         # estaciones excluidas tras un fallo
        self.paused = False          # requiere intervención del operador
        if gp('tasks_file'):
            self._load_tasks_file(gp('tasks_file'))

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lift = ActionClient(self, FollowJointTrajectory, '/lift_controller/follow_joint_trajectory')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel_nav', 10)
        self.fp_pubs = [self.create_publisher(Polygon, f'/{c}/footprint', 10)
                        for c in ('local_costmap', 'global_costmap')]
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/amr/task_status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/amr/stations_markers', latched)
        self.create_subscription(String, '/amr/task_request', self._on_request, 10)
        self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(f'{len(self.stations)} estaciones, {len(self.queue)} tareas en cola')

    # ================================================================ tareas
    def _load_tasks_file(self, path):
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
        self.home = cfg.get('home', self.home)
        for t in cfg.get('tasks', []) or []:
            self.queue.append({'pickup': t.get('pickup'), 'destination': t['destination']})
        if cfg.get('trips'):
            self._add_cycle(int(cfg['trips']), cfg.get('destinations', ['area_1']))

    def _add_cycle(self, n, areas):
        for i in range(n):
            self.queue.append({'pickup': None, 'destination': areas[i % len(areas)]})

    def _on_request(self, msg):
        txt = msg.data.strip()
        if txt in ('reanudar', 'resume'):
            self.paused = False
            self.blocked.clear()
            self.get_logger().info('Cola reanudada (estaciones desbloqueadas)')
            self._status()
            return
        with self.lock:
            try:
                if 'x ' in txt and txt.split('x ')[0].strip().isdigit():
                    n, areas = txt.split('x ', 1)
                    self._add_cycle(int(n), [a.strip() for a in areas.split(',') if a.strip()])
                elif '>' in txt:
                    pick, dest = (s.strip() for s in txt.split('>', 1))
                    self.queue.append({'pickup': pick, 'destination': dest})
                else:
                    self.queue.append({'pickup': None, 'destination': txt})
                self.get_logger().info(f'Petición "{txt}" añadida. Tareas en cola: {len(self.queue)}')
            except ValueError:
                self.get_logger().error(f'Petición no válida: "{txt}"')
        self._status()

    def _choose(self, task):
        """Asigna estación de recogida y de destino (y las reserva)."""
        with self.lock:
            pick = task['pickup']
            if pick is None:
                cands = [s for s in self.stations.values()
                         if s['area'] == PICKUP_AREA and s['loaded']
                         and s['id'] not in self.reserved | self.blocked]
                if not cands:
                    raise TaskError('no hay palets disponibles en el área de carga')
                pick = cands[0]['id']
            elif pick not in self.stations or not self.stations[pick]['loaded'] or pick in self.blocked:
                raise TaskError(f'la estación {pick} no existe, no tiene palet o está bloqueada')
            dests = [s for s in self.stations.values()
                     if s['area'] == task['destination'] and not s['loaded']
                     and s['id'] not in self.reserved | self.blocked]
            if not dests:
                raise TaskError(f'no hay huecos libres en {task["destination"]}')
            dest = dests[0]['id']
            self.reserved.update({pick, dest})
            return pick, dest

    # ================================================================ estado
    def _status(self, extra=None):
        done = sum(1 for h in self.history if h['ok'])
        failed = sum(1 for h in self.history if not h['ok'])
        msg = {'estado': self.state, 'tarea_actual': self.current, 'en_cola': len(self.queue),
               'completadas': done, 'fallidas': failed, 'cargado': self.loaded,
               'estaciones_bloqueadas': sorted(self.blocked), 'pausado': self.paused}
        if extra:
            msg.update(extra)
        self.status_pub.publish(String(data=json.dumps(msg, ensure_ascii=False)))

    def _set_state(self, state):
        self.state = state
        self.get_logger().info(f'[{self.current["id"] if self.current else "-"}] {state}')
        self._status()

    def _publish_markers(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, s in enumerate(self.stations.values()):
            x, y, ryaw = s['dock']
            rgb = AREA_RGB.get(s['area'], (0.7, 0.7, 0.7))
            box = Marker()
            box.header.frame_id, box.header.stamp = 'map', now
            box.ns, box.id, box.type, box.action = 'stations', i, Marker.CUBE, Marker.ADD
            box.pose.position.x, box.pose.position.y = x, y
            box.pose.position.z = 0.5 if s['loaded'] else 0.02
            _, _, qz, qw = yaw_to_quat(ryaw + math.pi / 2)    # palet: lado largo ⟂ al robot
            box.pose.orientation.z, box.pose.orientation.w = qz, qw
            box.scale.x, box.scale.y = 1.2, 0.8
            box.scale.z = 1.0 if s['loaded'] else 0.04
            box.color.r, box.color.g, box.color.b = rgb
            box.color.a = 0.85 if s['loaded'] else 0.35
            txt = Marker()
            txt.header = box.header
            txt.ns, txt.id, txt.type, txt.action = 'labels', i, Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y, 1.4
            txt.scale.z = 0.45
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = s['id'] + (' ■' if s['loaded'] else ' □')
            arr.markers += [box, txt]
        self.marker_pub.publish(arr)

    def _set_footprint(self, loaded):
        poly = Polygon(points=[Point32(x=float(a), y=float(b), z=0.0)
                               for a, b in (FOOTPRINT_LOADED if loaded else FOOTPRINT_EMPTY)])
        for pub in self.fp_pubs:
            pub.publish(poly)
        self.loaded = loaded

    # ============================================================ primitivas
    def _wait(self, future, timeout):
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                raise TaskError('tiempo de espera agotado')
            time.sleep(0.05)
        return future.result()

    def robot_pose(self):
        t = self.tf.lookup_transform('map', 'base_footprint', Time())
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def navigate(self, pose, label):
        retries = int(self.get_parameter('nav_retries').value)
        for attempt in range(retries + 1):
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
            _, _, qz, qw = yaw_to_quat(pose[2])
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = qz, qw
            handle = self._wait(self.nav.send_goal_async(goal), 10.0)
            if not handle.accepted:
                raise TaskError(f'Nav2 rechazó el objetivo {label}')
            res = self._wait(handle.get_result_async(), 600.0)
            if res.status == GoalStatus.STATUS_SUCCEEDED:
                return
            self.get_logger().warn(f'Navegación a {label} fallida (intento {attempt + 1}, '
                                   f'código {res.result.error_code})')
            time.sleep(2.0)
        raise TaskError(f'no se pudo navegar a {label}')

    def _cmd(self, vx=0.0, vy=0.0, wz=0.0):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_footprint'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(m)

    def move_to(self, target, max_speed, timeout=60.0, xy_tol=None, yaw_tol=None):
        """Movimiento fino holonómico en el frame map (alineación, atraque, salida)."""
        xy_tol = xy_tol or float(self.get_parameter('dock_xy_tolerance').value)
        yaw_tol = yaw_tol or float(self.get_parameter('dock_yaw_tolerance').value)
        t0 = time.time()
        ok_count = 0
        while time.time() - t0 < timeout:
            x, y, yaw = self.robot_pose()
            ex, ey, eyaw = target[0] - x, target[1] - y, wrap(target[2] - yaw)
            dist = math.hypot(ex, ey)
            if dist < xy_tol and abs(eyaw) < yaw_tol:
                ok_count += 1
                if ok_count >= 3:
                    self._cmd()
                    return
            else:
                ok_count = 0
            v = min(max_speed, 1.2 * dist)
            vx_m, vy_m = (ex / dist * v, ey / dist * v) if dist > 1e-6 else (0.0, 0.0)
            c, s = math.cos(yaw), math.sin(yaw)
            vx, vy = c * vx_m + s * vy_m, -s * vx_m + c * vy_m
            if 0.0 < v < 0.02:   # velocidad mínima para vencer el rozamiento
                vx, vy = vx * 0.02 / v, vy * 0.02 / v
            wz = max(-0.3, min(0.3, 1.5 * eyaw))
            self._cmd(vx, vy, wz)
            time.sleep(0.05)
        self._cmd()
        raise TaskError('movimiento fino: tiempo agotado')

    def set_lift(self, height):
        cur = self._lift_position()
        dur = max(abs(height - cur) / float(self.get_parameter('lift_speed').value), 1.0)
        pt = JointTrajectoryPoint(positions=[float(height)], velocities=[0.0],
                                  time_from_start=Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9)))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['lift_joint']
        goal.trajectory.points = [pt]
        handle = self._wait(self.lift.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise TaskError('el elevador rechazó la consigna')
        res = self._wait(handle.get_result_async(), dur + 30.0)
        if res.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise TaskError(f'fallo del elevador (código {res.result.error_code})')

    def _lift_position(self):
        try:
            t = self.tf.lookup_transform('base_link', 'lift_platform_link', Time())
            return t.transform.translation.z - 0.28   # z plegado de lift_joint
        except Exception:  # noqa: BLE001
            return 0.0

    def dock(self, st):
        self._set_state(f'alineando frente a {st["id"]}')
        self.move_to(st['pre_dock'], 0.10, timeout=40.0)
        self._set_state(f'entrando bajo {st["id"]}')
        self.docked_at = st
        self.move_to(st['dock'], float(self.get_parameter('dock_speed').value), timeout=60.0)

    def undock(self, st):
        self._set_state(f'saliendo de {st["id"]}')
        self.move_to(st['pre_dock'], float(self.get_parameter('dock_speed').value), timeout=60.0,
                     xy_tol=0.05, yaw_tol=0.05)
        self.docked_at = None

    def recover(self, failed_station):
        """Deja el robot en un estado seguro tras un fallo."""
        self._cmd()
        if failed_station is not None:
            self.blocked.add(failed_station)
        if self.docked_at is not None:
            st = self.docked_at
            try:
                if not self.loaded:
                    self._set_state(f'recuperación: bajando plataforma en {st["id"]}')
                    self.set_lift(0.0)
                self._set_state(f'recuperación: saliendo de {st["id"]}')
                self.undock(st)
            except TaskError as e:
                self.get_logger().error(f'Recuperación fallida: {e}')
                self.paused = True
        if self.loaded:
            # Con el palet elevado no se deposita en un lugar no previsto
            self.paused = True
        if self.paused:
            self._set_state('PAUSADO: requiere intervención del operador '
                            '(publicar "reanudar" en /amr/task_request)')

    # ================================================================ misión
    def run_task(self, task):
        pick_id, dest_id = self._choose(task)
        pick, dest = self.stations[pick_id], self.stations[dest_id]
        self.current = {'id': f'{pick_id}→{dest_id}', 'recogida': pick_id, 'destino': dest_id,
                        'area': task['destination']}
        self.phase_station = None
        try:
            self._set_state(f'navegando a recogida {pick_id}')
            self.phase_station = pick_id
            self.navigate(pick['pre_dock'], pick_id)
            self.dock(pick)
            self._set_state('elevando palet')
            self.set_lift(float(self.get_parameter('lift_transport_height').value))
            self._set_footprint(True)
            with self.lock:
                pick['loaded'] = False
            self.undock(pick)

            self._set_state(f'transportando a {dest_id} ({task["destination"]})')
            self.phase_station = dest_id
            self.navigate(dest['pre_dock'], dest_id)
            self.dock(dest)
            self._set_state('depositando palet')
            self.set_lift(0.0)
            with self.lock:
                dest['loaded'] = True
            self._set_footprint(False)
            self.undock(dest)
        finally:
            with self.lock:
                self.reserved.difference_update({pick_id, dest_id})

    def worker(self):
        self._set_footprint(False)
        self.state = 'esperando a Nav2'
        self._status()
        while rclpy.ok() and not self.nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Esperando a Nav2 (navigate_to_pose)...', throttle_duration_sec=10)
        self.lift.wait_for_server()
        while rclpy.ok():
            try:
                self.robot_pose()
                break
            except Exception:  # noqa: BLE001
                time.sleep(0.5)
        self.state = 'listo'
        self._status()
        at_home = True
        while rclpy.ok():
            if self.paused:
                time.sleep(0.5)
                continue
            with self.lock:
                task = self.queue.pop(0) if self.queue else None
            if task is None:
                if not at_home:
                    self.current = None
                    try:
                        self._set_state('volviendo a la base')
                        self.navigate(self.home, 'base')
                    except TaskError as e:
                        self.get_logger().error(f'Regreso a base: {e}')
                    at_home = True
                    self._set_state('en espera (sin tareas)')
                time.sleep(0.5)
                continue
            at_home = False
            t0 = self.get_clock().now()
            try:
                self.run_task(task)
                dt = (self.get_clock().now() - t0).nanoseconds * 1e-9
                self.history.append({'ok': True, 'tarea': self.current['id'], 'segundos': round(dt, 1)})
                self._set_state(f'tarea completada en {dt:.0f} s')
            except TaskError as e:
                self.history.append({'ok': False, 'tarea': self.current['id'] if self.current else task,
                                     'error': str(e)})
                self.get_logger().error(f'Tarea fallida: {e}')
                self._set_state(f'tarea fallida: {e}')
                self.recover(getattr(self, 'phase_station', None))
            self._status({'ultimo': self.history[-1]})


def main():
    rclpy.init()
    node = TaskManager()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=node.worker, daemon=True).start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
