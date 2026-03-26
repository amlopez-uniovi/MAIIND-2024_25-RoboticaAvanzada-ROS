# turtlesim_control - ROS 2 Control de Movimiento en Turtlesim

Este paquete de ROS 2 implementa dos tipos de control para `turtlesim`:

- **Movimiento en círculo**
- **Movimiento hacia un punto objetivo (go to goal)**

Ambos usan:

- `/turtle1/cmd_vel` → control de velocidad
- `/turtle1/pose` → lectura de posición

---

## Creación del workspace

```bash
mkdir -p ~/maiind_ws/src
cd ~/maiind_ws
colcon build
source install/setup.bash
```

## Creación del paquete

```bash
cd ~/maiind_ws/src
ros2 pkg create --build-type ament_python turtlesim_control --dependencies rclpy geometry_msgs turtlesim
mkdir -p turtlesim_control/resource
touch turtlesim_control/resource/turtlesim_control
```

## Crear archivos de control

```bash
cd ~/maiind_ws/src/turtlesim_control/turtlesim_control
touch turtlesim_circle_node.py
touch go_to_goal_node.py
```

```bash
cd ~/maiind_ws/src/turtlesim_control
mkdir -p launch
touch launch/circle_launch.py
touch launch/go_to_goal_launch.py
```

## Estructura del proyecto

```text
turtlesim_control/
├── resource/
│   └── turtlesim_control
├── turtlesim_control/
│   ├── __init__.py
│   ├── turtlesim_circle_node.py
│   └── go_to_goal_node.py
├── launch/
│   ├── circle_launch.py
│   └── go_to_goal_launch.py
├── package.xml
└── setup.py
```

## Nodo 1: Movimiento en círculo (turtlesim_circle_node.py)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class CircleTurtle(Node):
    def __init__(self):
        super().__init__('circle_turtle')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move)
        self.get_logger().info("Movimiento en círculo iniciado")

    def move(self):
        twist = Twist()
        twist.linear.x = 2.0
        twist.angular.z = 1.0
        self.publisher.publish(twist)

    def pose_callback(self, msg):
        self.get_logger().info(f'x: {msg.x:.2f}, y: {msg.y:.2f}, theta: {msg.theta:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CircleTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Nodo 2: Go To Goal (go_to_goal_node.py)

```python
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.move)
        self.goal_x = 10.0
        self.goal_y = 10.0
        self.pose = None
        self.get_logger().info("Moviendo hacia el objetivo")

    def pose_callback(self, msg):
        self.pose = msg

    def move(self):
        if self.pose is None:
            return

        twist = Twist()
        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)
        error = angle - self.pose.theta

        twist.linear.x = 1.5 * distance
        twist.angular.z = 4.0 * error

        if distance < 0.1:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Objetivo alcanzado")

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch files

### `circle_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node'),
        Node(package='turtlesim_control', executable='circle'),
    ])
```

### `go_to_goal_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node'),
        Node(package='turtlesim_control', executable='go_to_goal'),
    ])
```

## `setup.py`

```python
from setuptools import setup

package_name = 'turtlesim_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/circle_launch.py',
            'launch/go_to_goal_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@example.com',
    description='Control de movimiento en turtlesim',
    license='MIT',
    entry_points={
        'console_scripts': [
            'circle = turtlesim_control.turtlesim_circle_node:main',
            'go_to_goal = turtlesim_control.go_to_goal_node:main',
        ],
    },
)
```

## Compilar

```bash
cd ~/maiind_ws
colcon build
source install/setup.bash
```

## Ejecución por línea de comandos

### Círculo

#### Terminal 1

```bash
ros2 run turtlesim turtlesim_node
```

#### Terminal 2

```bash
ros2 run turtlesim_control circle
```

### Go To Goal

#### Terminal 1

```bash
ros2 run turtlesim turtlesim_node
```

#### Terminal 2

```bash
ros2 run turtlesim_control go_to_goal
```

## Ejecución con launch

### Círculo

```bash
ros2 launch turtlesim_control circle_launch.py
```

### Go To Goal

```bash
ros2 launch turtlesim_control go_to_goal_launch.py
```
