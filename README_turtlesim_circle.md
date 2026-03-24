# turtlesim_circle - ROS 2 Demo con Movimiento Circular

Este paquete de ROS 2 controla el nodo `turtlesim` para que la tortuga realice un movimiento circular continuo, mientras muestra su posición actual en consola. 

## Creación de entorno de trabajo

Primero, creamos un entorno de trabajo para almacenar nuestros desarrollos.

```bash
mkdir -p ~/maiind_ws/src
cd ~/maiind_ws
colcon build
source install/setup.bash
```

## Creación del paquete: turtlesim_circle

## Estructura del proyecto

```bash
turtlesim_circle/
├── resource/
│   └── turtlesim_circle
├── turtlesim_circle/
│   └── __init__.py
    └── turtlesim_circle_node.py
├── launch/
│   └── turtlesim_circle_launch.py
├── package.xml
└── setup.py
```

## Objetivo del nodo

- Controlar la tortuga de `turtlesim` para que se mueva en un **círculo constante**.
- Imprimir en pantalla su **posición actual** (`x`, `y`, `theta`) en tiempo real.

## Creación del paquete

Primero, crea la estructura básica del paquete.

```bash
cd ~/maiind_ws/src
ros2 pkg create --build-type ament_python turtlesim_circle --dependencies rclpy geometry_msgs turtlesim
```

Luego, crea el archivo de recursos requerido por ROS 2 para registrar el nombre del paquete:

```bash
mkdir -p turtlesim_circle/resource
touch turtlesim_circle/resource/turtlesim_circle
```

## 🐍 Nodo principal: `turtlesim_circle_node.py`

```bash
cd ~/maiind_ws/src/turtlesim_circle/turtlesim_circle
touch turtlesim_circle_node.py
```

Modifica el contenido del archivo turtlesim_circle_node.py con el siguiente codigo
```python
# turtlesim_circle/turtlesim_circle_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class CircleTurtle(Node):
    def __init__(self):
        super().__init__('circle_turtle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move_in_circle)
        self.get_logger().info("Nodo iniciado: la tortuga se moverá en círculo.")
        
    def move_in_circle(self):
        twist = Twist()
        twist.linear.x = 2.0      # velocidad lineal
        twist.angular.z = 1.0     # velocidad angular (giro)
        self.publisher.publish(twist)

    def pose_callback(self, msg):
        self.get_logger().info(f'Posición actual -> x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = CircleTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🔧 `setup.py`

Asegúrate de incluir la información para instalar el nodo modificando el archivo setup.py como sigue (en realidad solo tendrás que modificar los entry_points):

```python
from setuptools import setup

package_name = 'turtlesim_circle'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@example.com',
    description='Un paquete ROS 2 para hacer que turtlesim se mueva en círculo y muestre su posición.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_turtle = turtlesim_circle.turtlesim_circle_node:main',
        ],
    },
)
```
Una vez hecho reconstruye el nodo:

```bash
cd ~/maiind_ws
colcon build
source install/setup.bash
```
## ▶️ Ejecutar el nodo

Primero, abre dos terminales. En el primero:

```bash
ros2 run turtlesim turtlesim_node
```

Y en el segundo:

```bash
ros2 run turtlesim_circle circle_turtle
```


## 🚀 Lanzamiento del nodo

Puedes crear un archivo de lanzamiento opcional en `launch/turtlesim_circle_launch.py` para iniciar `turtlesim_node` y `circle_turtle` juntos:

```bash
mkdir -p ~/maiind_ws/src/turtlesim_circle/launch
cd ~/maiind_ws/src/turtlesim_circle/launch
touch turtlesim_circle_launch.py
```

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtlesim_circle',
            executable='circle_turtle',
            name='circle_turtle'
        )
    ])
```

El archivo setup.py debe modificarse para añadir lo siguiente:

```python
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Aquí se instala el archivo de launch
        ('share/' + package_name + '/launch', ['launch/turtlesim_circle_launch.py']),
    ],
```

## ⚙️ Compilar e instalar

Desde el workspace raíz:

```bash
cd ~/maiind_ws
colcon build
source install/setup.bash
```
Con la tortuga cerrada, al lanzar el siguiente comando se activan los dos nodos simultáneamente.

```bash
ros2 launch turtlesim_circle turtlesim_circle_launch.py
```
