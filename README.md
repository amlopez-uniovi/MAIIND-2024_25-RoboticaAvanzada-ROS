# MAIIND-2024_25-RoboticaAvanzada-ROS

Repositorio para el módulo de ROS de la asignatura de Robótica Avanzada en el Máster en Ingeniería de Automatización e Informática Industrial por la Universidad de Oviedo.

Este repositorio contiene las instrucciones necesarias para configurar un entorno de trabajo basado en Docker y ROS 2 Humble, así como ejemplos prácticos para familiarizarse con el uso de ROS 2.

## Creación de la imagen de Docker

Para crear la imagen de Docker que utilizaremos en este módulo, sigue estos pasos:

1. Asegúrate de estar en la carpeta raíz del repositorio.
2. Ejecuta el siguiente comando:

    ```bash
    docker build -t ros2-humble-maiind .
    ```

    **Explicación:**
    - `docker build`: Comando para construir una imagen de Docker.
    - `-t ros2-humble-maiind`: Etiqueta (nombre) asignada a la imagen creada.
    - `.`: Indica que el contexto de construcción es la carpeta actual.

## Crear el repositorio de trabajo

1. Crea una carpeta para el workspace de ROS 2:

    ```bash
    mkdir -p ./maiind_ws/src
    ```

    **Explicación:**
    - `mkdir -p`: Crea directorios, incluyendo los intermedios si no existen.
    - `./maiind_ws/src`: Ruta del workspace y su subdirectorio `src`.

2. Ejecuta el contenedor de Docker con el siguiente comando:

    ```bash
    docker run -it -v $(pwd)/maiind_ws/src:/root/maiind_ws/src --name maiind_ros2_humble ros2-humble-maiind
    ```

    **Explicación:**
    - `docker run`: Crea y ejecuta un contenedor basado en una imagen.
    - `-it`: Permite la interacción con el contenedor a través de un terminal.
    - `-v $(pwd)/maiind_ws/src:/root/maiind_ws/src`: Monta un volumen para sincronizar la carpeta local `maiind_ws/src` con el contenedor.
    - `--name maiind_ros2_humble`: Asigna un nombre al contenedor.
    - `ros2-humble-maiind`: Nombre de la imagen utilizada para crear el contenedor.

## Comandos útiles

### Salir del contenedor

Para salir del shell del contenedor, utiliza el comando:

```bash
exit
```

### Abrir otro shell en el contenedor en ejecución

Si el contenedor está en ejecución y necesitas abrir otro terminal, usa:

```bash
docker exec -it maiind_ros2_humble bash
```

**Explicación:**
- `docker exec`: Ejecuta un comando en un contenedor en ejecución.
- `-it`: Permite la interacción con el terminal del contenedor.
- `ros2-humble-maiind`: Nombre del contenedor.
- `bash`: Shell que se abrirá en el contenedor.

### Inspeccionar el enlace con el volumen local

Para verificar cómo está configurado el volumen entre tu máquina y el contenedor:

```bash
docker inspect maiind_ros2_humble | grep -i "Mounts" -A 10
```

**Explicación:**
- `docker inspect`: Muestra información detallada sobre el contenedor.
- `grep -i "Mounts" -A 10`: Filtra la salida para mostrar información sobre los volúmenes.

### Iniciar un contenedor detenido

Si el contenedor ha sido detenido, puedes iniciarlo nuevamente con:

```bash
docker start maiind_ros2_humble
```

**Explicación:**
- `docker start`: Inicia un contenedor detenido.
- `ros2-humble-maiind`: Nombre del contenedor.

### Detener el contenedor

Para detener el contenedor en ejecución:

```bash
docker stop maiind_ros2_humble
```

**Explicación:**
- `docker stop`: Detiene un contenedor en ejecución.
- `ros2-humble-maiind`: Nombre del contenedor.

## Prueba de turtlesim

`turtlesim` es una herramienta gráfica que permite practicar conceptos básicos de ROS 2. Sigue estos pasos para probarlo:

1. **Instalar un servidor X11**  
    Necesitarás un servidor X11 para visualizar aplicaciones gráficas desde el contenedor. Ejemplos:
    - **Mac**: [XQuartz](https://www.xquartz.org/)
        - Ir al menú *XQuartz > Preferences > Security*, marcar la opción *Allow connections from network clients*.
        - Cerrar XQuartz y reiniciar
        - En una terminal de mac ejecutar el comando *xhost + 127.0.0.1*
    - **Windows**: [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/)
        - Instalará la aplicación *XLaunch*.
        - Al iniciarla seleccionar los *Display Settings*: Seleccionar *Multplie Windows* y poner 0 en *Display Number*.

2. **Ejecutar el nodo de turtlesim**  
    Abre una terminal en el contenedor y ejecuta:

    ```bash
    ros2 run turtlesim turtlesim_node
    ```

    **Explicación:**
    - `ros2 run`: Ejecuta un nodo de ROS 2.
    - `turtlesim`: Paquete que contiene el nodo.
    - `turtlesim_node`: Nodo que inicia la simulación gráfica.

3. **Publicar comandos de movimiento**  
    Abre una segunda terminal en el contenedor y ejecuta:

    ```bash
    ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
    ```

    **Explicación:**
    - `ros2 topic pub`: Publica mensajes en un tópico.
    - `/turtle1/cmd_vel`: Tópico donde se envían comandos de velocidad.
    - `geometry_msgs/msg/Twist`: Tipo de mensaje utilizado.
    - `"{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"`: Mensaje que indica velocidad lineal y angular.

4. **Observar la posición de la tortuga**  
    Abre una tercera terminal en el contenedor y ejecuta:

    ```bash
    ros2 topic echo /turtle1/pose
    ```

    **Explicación:**
    - `ros2 topic echo`: Muestra los mensajes publicados en un tópico.
    - `/turtle1/pose`: Tópico que publica la posición de la tortuga.

5. **Controlar la tortuga con el teclado**  
    Cancela los comandos en las terminales 2 y 3. Luego, en una de ellas, ejecuta:

    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```

    **Explicación:**
    - `ros2 run`: Ejecuta un nodo de ROS 2.
    - `turtlesim`: Paquete que contiene el nodo.
    - `turtle_teleop_key`: Nodo que permite controlar la tortuga con las teclas de dirección.
