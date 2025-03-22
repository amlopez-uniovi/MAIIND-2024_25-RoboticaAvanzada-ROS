# MAIIND-2024_25-RoboticaAvanzada-ROS
Repositorio para el módulo de ROS de la asignatura de Robótica Avanzada en el Máster en Ingeniería de Automatización e Informática Industrial por la Universidad de Oviedo

## Creación de la imagen de Docker

Situarse en la carpeta de trabajo (esta) y ejecutar el comando:

```
docker build -t ros2-foxy-maiind  .
```

## Crear el repositorio

```
mkdir -p ./maiind_ws/src

docker run -it -v $(pwd)/maiind_ws/src:/root/maiind_ws/src --name maiind_ros2 ros2-foxy-maiind

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
``
Para para el contendor:
```
docker stop maiind_ros2
```



