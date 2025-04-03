# MAIIND-2024_25-RoboticaAvanzada-ROS
Repositorio para el módulo de ROS de la asignatura de Robótica Avanzada en el Máster en Ingeniería de Automatización e Informática Industrial por la Universidad de Oviedo

## Creación de la imagen de Docker

Situarse en la carpeta de trabajo (esta) y ejecutar el comando:

```
docker build -t ros2-humble-maiind  .
```

## Crear el repositorio

```
mkdir -p ./maiind_ws/src

docker run -it -v $(pwd)/maiind_ws/src:/root/maiind_ws/src --name maiind_ros2_humble ros2-humble-maiind

```

### Algunos comando útiles
Para salir del shell del contenedor usar el comando *exit*.

Si queremos abrir otro shell del contenedor, cuando este está en ejecución:
```
docker exec -it maiind_ros2 bash
```
Si desde una terminal en nuestro equipo queremos inspeccionar el enlace con el volumen local:
```
docker inspect maiind_ros2 | grep -i "Mounts" -A 10
```
Para ejecutar el contenedor si lo hemos detenido:
```
docker start maiind_ros2
```
Para para el contendor:
```
docker stop maiind_ros2
```

# 🐢 turtlesim_circle - ROS 2 Nodo para mover la tortuga en círculos

Este paquete de ROS 2 crea un nodo en Python que mueve la tortuga del simulador `turtlesim` en círculos y muestra su posición en consola en tiempo real.

---

## 📦 Requisitos

- ROS 2 Humble, Foxy o compatible
- Python 3
- turtlesim (`sudo apt install ros-<distro>-turtlesim`)

---

## 🚀 Instalación

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash

# Crear el paquete

cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python turtlesim_circle --dependencies rclpy turtlesim

# Código del nodo

cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python turtlesim_circle --dependencies rclpy turtlesim


## Contenido de turtle_circle_node.py:

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleCircle(Node):

    def __init__(self):
        super().__init__('turtle_circle_node')

        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.move_turtle)

        self.get_logger().info('Nodo turtle_circle_node iniciado.')

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)

    def pose_callback(self, msg):
        self.get_logger().info(f'Posición -> x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Editar setup.py

Edita setup.py para añadir el punto de entrada:
python
CopiarEditar
entry_points={
    'console_scripts': [
        'turtle_circle = turtlesim_circle.turtle_circle_node:main',
    ],
},

# Compilar

bash
CopiarEditar
cd ~/ros2_ws
colcon build
source install/setup.bash

# Ejecutar
En una terminal:

ros2 run turtlesim turtlesim_node

En otra terminal:

source ~/ros2_ws/install/setup.bash
ros2 run turtlesim_circle turtle_circle



