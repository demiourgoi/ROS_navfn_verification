# pubsub_ros
Ejemplo mínimo de nodos ROS2 que se comunican a través de un _topic_ y cada uno reliza reducción de términos Maude

## Prerrequisitos
1. Instalar ROS2 en el sistema. Yo he usado la versión **dashing** en Ubuntu 18.04 y es sencillo (https://index.ros.org/doc/ros2/Installation/Dashing/).
2. Tener la biblioreca Python que hizo Rubén y que sirve para conectar Python con Maude. Se instala con `pip install maude` (¡viva Rubén!). Si falla la instalación en Ubuntu 18.04 probad a actualizar `pip` (a mi no me encontraba ninguna versión para mi arquitectura usando la versión `pip` oficial en los repositorios de Ubuntu 18.04 hasta que lo actualicé).

## Compilación
1. `source /opt/ros/dashing/setup.bash`
1. `colcon build --symlink-install`

## Ejecución
1. `. install/setup.bash`
2. `source set_env.sh`
3. Lanzar el _publisher_ con `ros2 run pubsub_maude_ros mauder_talker`
4. Lanzar el _subscriber_ en otro terminal con `ros2 run pubsub_maude_ros mauder_listener` (se deben ejecutar los pasos 1 y 2)
