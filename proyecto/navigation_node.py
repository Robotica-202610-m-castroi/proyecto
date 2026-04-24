import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .logic.grid import discretizar_cspace
from .logic.pathfinding import astar

import math
import threading
import os

from .logic.lidar import obtener_distancia_angulo, obtener_distancias_rango
from .logic.movement import calcular_rotacion, calcular_movimiento_relativo
from .logic.trajectory import *

from .models.scene import Scene
from .models.Cspace import CSpace

from .logic.plots import *

import time

class NavigationNode(Node):
    def __init__(self):
        super().__init__('student_navigation')
        
        # Suscriptores
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, 'scan_raw', self.lidar_callback, 10)
        
        # Publicador
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Variables de estado interno
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_scan = None
        
        # Memorias de estado para los movimientos relativos
        self.target_theta_relativo = None
        self.pose_inicial_relativa = None 
        
        # Variable para almacenar el texto crudo de la escena
        self.texto_escena = None
        
        # Variables para comunicar el menú interactivo con el control loop
        self.comando_activo = None
        self.parametros_comando = []
        
        # Timer (El loop de control corre 10 veces por segundo)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nodo de Navegación Estudiantil Iniciado.")

        # Iniciamos el menú en un hilo separado para NO bloquear a ROS2
        self.hilo_menu = threading.Thread(target=self.menu_interactivo, daemon=True)
        self.hilo_menu.start()
        
         # Signal to wait for command completion
        self.command_done = threading.Event()
        
        self.scene = None
        self.movements = None
        
        self.blocked = False

    # =======================================================
    # CALLBACKS DE ROS2
    # =======================================================
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.current_theta = 2.0 * math.atan2(qz, qw)

    def lidar_callback(self, msg):
        self.last_scan = msg

    # =======================================================
    # WRAPPERS PARA LOS ESTUDIANTES
    # =======================================================
    def leer_distancia_en_angulo(self, grados):
        """Retorna la distancia (en metros) de un ángulo específico del Lidar."""
        return obtener_distancia_angulo(self.last_scan, math.radians(grados))

    def leer_distancia_direccion(self, direccion):
        """
        Retorna la distancia en una dirección cardinal específica:
        'frente', 'atras', 'izquierda', 'derecha'.
        """
        mapa_direcciones = {
            'frente': 0.0,
            'izquierda': 90.0,
            'derecha': 270.0,
            'atras': 180.0
        }
        
        direccion = direccion.lower()
        if direccion in mapa_direcciones:
            return self.leer_distancia_en_angulo(mapa_direcciones[direccion])
        else:
            self.get_logger().error(f"Dirección '{direccion}' no válida.")
            return float('inf')

    def leer_distancias_en_rango(self, grados_min, grados_max):
        """Retorna una lista con todas las detecciones en un rango visual."""
        return obtener_distancias_rango(self.last_scan, grados_min, grados_max)

    def rotar_relativo(self, grados_relativos, tolerancia=0.05):
        """
        Gira el robot de forma relativa (ej: 90 grados a la izquierda).
        Retorna True si la maniobra ya finalizó.
        """
        if self.target_theta_relativo is None:
            # Capturamos el ángulo base al arrancar la maniobra
            self.target_theta_relativo = self.current_theta + math.radians(grados_relativos)
            
        cmd, completado = calcular_rotacion(self.current_theta, self.target_theta_relativo, tolerancia=tolerancia)
        self.cmd_pub.publish(cmd)
        
        if completado:
            # Reseteamos la meta para que el robot pueda volver a girar en el futuro
            self.target_theta_relativo = None 
            
        return completado

    def mover_relativo(self, distancia_x_metros, distancia_y_metros, cono_vision=30, dist_segura=0.1, vel_lineal=0.4):
        """
        Desplazamiento usando Cinemática de Tiempo (Dead Reckoning).
        Evalúa el obstáculo según la dirección (Adelante, Atrás, Lados).
        """
        # 1. Inicializamos el cronómetro al arrancar la orden
        if self.pose_inicial_relativa is None:
            self.pose_inicial_relativa = True # Lo usamos como bandera de inicio
            self.tiempo_maniobra = 0.0        # Cronómetro en segundos

        # 2. Determinar hacia dónde nos vamos a mover para vigilar ESA dirección
        if abs(distancia_x_metros) >= abs(distancia_y_metros):
            if distancia_x_metros >= 0:
                # Movimiento hacia el FRENTE
                cono_despejado = self.leer_distancias_en_rango(-cono_vision, cono_vision)
            else:
                # Movimiento hacia ATRÁS (El Lidar ROS2 va de -180 a 180, unimos los dos extremos)
                cono_despejado = self.leer_distancias_en_rango(180-cono_vision, 180) + self.leer_distancias_en_rango(-180, -180+cono_vision)
        else:
            if distancia_y_metros > 0:
                # Movimiento hacia la IZQUIERDA
                cono_despejado = self.leer_distancias_en_rango(90-cono_vision, 90+cono_vision)
            else:
                # Movimiento hacia la DERECHA
                cono_despejado = self.leer_distancias_en_rango(270-cono_vision, 270+cono_vision)

        # 3. Calcular movimiento basado en tiempo
        cmd, estado = calcular_movimiento_relativo(
            self.tiempo_maniobra,
            distancia_x_metros, distancia_y_metros,
            cono_despejado,
            dist_segura=dist_segura,
            vel_lineal=vel_lineal
        )
        self.cmd_pub.publish(cmd)
        
        # 4. Sumamos el tiempo de este ciclo (nuestro timer general corre a 0.1s)
        self.tiempo_maniobra += 0.1
        
        # 5. Si terminamos o nos bloqueamos, limpiamos todo para el próximo comando
        if estado in ['COMPLETADO', 'BLOQUEADO']:
            self.pose_inicial_relativa = None
            self.tiempo_maniobra = 0.0
            
        return estado

    def cargar_escena(self, numero_escena):
        """
        Lee el archivo de la escena indicada y guarda el texto en self.texto_escena.
        """
        # Calculamos la ruta subiendo un nivel de directorio desde este archivo hasta la carpeta 'data'
        # workspace_path = os.path.expanduser('~/proyecto')
        directorio_actual = os.path.dirname(os.path.abspath(__file__))
        ruta_archivo = os.path.join(directorio_actual, '..', 'data', f'Escena-Problema{numero_escena}.txt')
        # ruta_archivo = os.path.join(workspace_path, 'data', f'Escena-Problema{numero_escena}.txt')
        
        try:
            with open(ruta_archivo, 'r', encoding='utf-8') as archivo:
                self.texto_escena = archivo.read()
            self.get_logger().info(f"Escena {numero_escena} cargada correctamente.")
            # Opcional: imprimir un pedacito para confirmar
            print(f"\n--- Contenido Escena {numero_escena} ---\n{self.texto_escena}\n---------------------------")
        except FileNotFoundError:
            self.get_logger().error(f"No se encontró el archivo: {ruta_archivo}")
        except Exception as e:
            self.get_logger().error(f"Error al leer la escena: {e}")
    
    def parse_scene_text(self, text:str):
        scene = {}
        for i, line in enumerate(text.splitlines()):
            data = line.split(',')
            scene[data[0].lower()] = list(map(float, data[1:]))
        return scene
    
    def get_start_goal_cell(self, res):
        qi = self.scene.conf_init
        qf = self.scene.conf_final
        
        theta0 = math.radians(qi.theta)
        thetaf = math.radians(qf.theta)

        x0 = qi.point.x
        y0 = qi.point.y

        xf = qf.point.x
        yf = qf.point.y

        start = (int(y0 / res), int(x0 / res))
        goal  = (int(yf / res), int(xf / res))
        return start, goal
        

    # =======================================================
    # BUCLE PRINCIPAL (Área de trabajo del estudiante)
    # =======================================================
    def menu_interactivo(self):
        """Pide input por consola sin interrumpir la recepción de datos de los sensores."""
        while rclpy.ok():
            if self.texto_escena is None:
                print("\n" + "="*35)
                print("--- CARGAR ESCENA DE TEXTO ---")

                print("="*35)
                
                try:
                    numero = int(input("Ingresa el número de la escena (1-6): "))
                    self.cargar_escena(numero_escena=numero)
                    if self.texto_escena:
                        raw_scene = self.parse_scene_text(self.texto_escena)
                        self.scene = Scene(raw_scene)
                        print("Escena cargada, comenzando planificación...")
            
                    if self.scene and not hasattr(self, 'ya_grafico'):
                        self.ya_grafico = True
                    
                    if self.scene:
                        # ======================================
                        # 1. Construir el C-Espacio con theta=0
                        # ======================================
                        print(self.scene.__repr__())
                        
                        cspace = CSpace(
                            self.scene.robot_geom, 
                            self.scene.obstacles, 
                            0
                        )
                        
                        configure_plot()
                        
                        plot_polygon(
                            [[0,0], [self.scene.dimension[0], 0], [*self.scene.dimension], [0, self.scene.dimension[1]]], 'y:', thickness=5, 
                            labelstr='Frontera W-Space / C-Space'
                        )
                        
                        plot_polygon(
                            cspace.robot_rotated.points + np.array(self.scene.conf_init_r.conf[:2]),
                            color='b--', 
                            labelstr=f'Robot (rotado θ={self.scene.conf_init.theta})'
                        )
                        
                        for obs in cspace.obstacles_geom:
                            plot_polygon(obs.points, 'r-', labelstr=f'Obstáculo {obs.id}')
                    
                        for idx, cobs in enumerate(cspace.c_obstacles):
                            plot_polygon(cobs, 'g-', labelstr=f'C-Obstáculo {idx + 1}')
                        
                        # ======================================
                        # 2. Discretizar el espacio de trabajo
                        # ======================================
                        
                        res = 0.2
                        grid, cells = discretizar_cspace(self.scene, cspace, resolucion=res)
                        
                        # plot_cell_classification(cells, grid)

                        # ================================
                        # 3. Calcular ruta - algoritmo A*
                        # ================================
                        
                        start, goal = self.get_start_goal_cell(res)
                        path = astar(grid, start, goal)
                        if path:
                            print("Ruta encontrada")
                            print(path)
                            
                            plot_path(
                                path, 
                                self.scene.conf_init.conf[:2],
                                self.scene.conf_final.conf[:2],
                                res
                            )
                            
                            theta0 = math.radians(self.scene.conf_init.theta)
                            plot_robot_orientation(
                                self.scene.conf_init_r.conf[:2], 
                                theta0 
                            )
                            
                            thetaf = math.radians(self.scene.conf_final_r.theta)
                            plot_robot_orientation(
                                self.scene.conf_final.conf[:2], 
                                thetaf
                            )
                            
                            
                            # ================================
                            # 4. Ejecutar trayectoria
                            # ================================
                            
                            configs_list = define_trayectory_configs(cells, path, self.scene.conf_final_r)

                            configs_list.insert(0, self.scene.conf_init_r)

                            output_path = os.path.join(
                                os.path.dirname(os.path.abspath(__file__)),
                                '..',
                                'out',
                                f"{numero}_configuraciones_escena.txt"
                            )
                            
                            with open(output_path, 'w', encoding='utf-8') as f:
                                for c in configs_list:
                                    f.write(f"{c}\n")
                        
                            self.get_logger().info(f"Configuraciones guardadas en: {output_path}")
                            
                            plot_trayectory(configs_list)
                            save(numero)
                            
                            self.movements = define_trayectory_movements(configs_list)
                            output_path = os.path.join(
                                os.path.dirname(os.path.abspath(__file__)),
                                '..',
                                'out',
                                f"{numero}_trayectorias_escena.txt"
                            )
                            
                            with open(output_path, 'w', encoding='utf-8') as f:
                                for m in self.movements:
                                    f.write(f"{m}\n")
                                    
                            self.get_logger().info(f"Movimientos guardados en: {output_path}")
                            
                            # --> 4.2 Ejectuar trayectoria
                            self.get_logger().info("Comenzando ejecución de trayectoria")

                            for i, mov in enumerate(self.movements):
                                if self.blocked:
                                    self.get_logger().warn("Se paró el robot, saliendo de ejecucción de trayectoria.")
                                    break
                                
                                if mov.is_rotation:
                                    print(f"Rotando {mov.da}")
                                    self.parametros_comando = [mov.da]
                                    self.comando_activo = 3
                                else:
                                    print(f"{i}: Desplazando x, y {[mov.forward, 0]}")
                                    self.parametros_comando = [mov.forward, 0]
                                    self.comando_activo = 4
                                
                                print(f"curr.x={self.current_x} | curr.y={self.current_y}")
                                self.command_done.wait()
                                self.command_done.clear()
                                
                                time.sleep(1)

                            self.get_logger().info("FIN de trayectoria")
                            
                            # ================================
                            # 5. Cálculo de configuraciones
                            # ================================
                            q_teo = self.scene.conf_final
                            q_est = Configuration(
                                self.scene.conf_init_r.point.x + self.current_x + .15,
                                self.scene.conf_init_r.point.y + self.current_y + .15,
                                self.scene.conf_init_r.theta + self.current_theta,
                            )
                            
                            
                            d_frente = self.leer_distancia_direccion('frente')
                            d_izq = self.leer_distancia_direccion('izquierda')

                            print(f"curr.x={self.current_x} | curr.y={self.current_y}")
                            print(q_est.__str__())
                            print(f"f={d_frente}, i={d_izq}")
                            
                            
                        else:
                            print("No se encontró ruta")
                        
                except FileNotFoundError as e:
                    print("El archivo deseado no existe.")    
                    print(e)
                except ValueError as e:
                    print("Por favor, ingresa únicamente números válidos.")
                    print(e)
                except Exception as e:
                    print(f"Error al cargar la escena: {e}")

    # =======================================================
    # BUCLE PRINCIPAL DE CONTROL
    # =======================================================
    def control_loop(self):
        
        # Evitar fallos si no hay datos del sensor todavía
        if self.last_scan is None:
            pass
        
        if self.comando_activo == 3:
            if self.rotar_relativo(self.parametros_comando[0]):
                self.get_logger().info("Rotación completada exitosamente.")
                self.comando_activo = None
                self.command_done.set()  # Signal next command
                
        elif self.comando_activo == 4:
            estado = self.mover_relativo(self.parametros_comando[0], self.parametros_comando[1])
            
            if estado == 'COMPLETADO':
                self.get_logger().info("Desplazamiento relativo completado.")
                self.comando_activo = None
                self.command_done.set()  
                time.sleep(0.5)
            elif estado == 'BLOQUEADO':
                self.get_logger().warn("¡Obstáculo detectado! Ruta bloqueada. Abortando movimiento.")
                self.blocked = True
                self.comando_activo = None
                time.sleep(0.5)
                self.command_done.set()  

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Detener motores forzosamente si se presiona Ctrl+C
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()