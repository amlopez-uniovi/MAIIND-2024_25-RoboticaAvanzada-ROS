# MAIIND-2024_25-RoboticaAvanzada-ROS
Repositorio para el módulo de ROS de la asignatura de Robótica Avanzada en el Máster en Ingeniería de Automatización e Informática Industrial por la Universidad de Oviedo

## Creación de la imagen de Docker

Situarse en la carpeta de trabajo (esta) y ejecutar el comando:

```
docker build -t ros-foxy-turtlesim  .
```

## Crear el repositorio

```
mkdir -p ./maiind_ws/src

docker run -it -v $(pwd)/maiind_ws/src:/root/maiind_ws/src --name maiind_ros ros-foxy-turtlesim

```

### Algunos comando útiles

Tras ejecutar el \em run, estaremos en una terminal dentro del shell del contenedor. Si queremos salir, teclear comando \em exit

Si queremos inspeccionar el enlace con el volumen local:
```
docker inspect maiind_ros | grep -i "Mounts" -A 10
```
Si queremos volver al shel del contenedor:
```
docker exec -it maiind_ros bash
```



