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
### Prueba de turtlesim

Necesitarás instalar un servidor X11 en tu operativo como XQuartz (Mac) o VcXsrv Windows X Server (Windows, ver PL1_Entorno de Trabajo.pdf con info relacionada).

Abre una terminal en el contenedor y ejecuta:
```
ros2 run turtlesim turtlesim_node
```
Abre una segunda terminal en el contenedor y ejecuta:
````
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
````

Abre una tercera terminal en el contenedor y ejecuta:
````
ros2 topic echo /turtle1/pose
````
Cancela la ejecución en las termianles 2 y 3. En una de ellas ejecuta el siguiente comando, que te permitirá manejar la tortuga con los cursores del teclado:
```
ros2 run turtlesim turtle_teleop_key
```