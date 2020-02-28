# pubsub_ros
Ejemplo mínimo de nodos ROS2 que se comunican a través de un _topic_ y cada uno reliza reducción de términos Maude

## Prerrequisitos
1. Instalar ROS2 en el sistema. Yo he usado la versión **dashing** en Ubuntu 18.04 y es sencillo (https://index.ros.org/doc/ros2/Installation/Dashing/).
2. Tener la biblioreca Python que hizo Rubén y que sirve para conectar Python con Maude. Yo probé el release 0.1 de https://github.com/fadoss/maude-bindings/releases y no me funcionaba en Ubuntu 18.04 por diferencias de versiones de bibliotecas (libreadline, glibc, etc.). Así que compilé el proyecto a partir de sus fuentes (https://github.com/fadoss/maude-bindings) con `meson` (necesité la última versión de `pip`, no la del repositorio porque era antigua) y `ninja`. Luego fui a la carpeta `maude` del release 0.1 y actualicé `libmaude.so` y `_maude` por la versiones con las bibliotecas bien enlazadas. 
3. Establecer algunas variables de entorno para que se encuentre la biblioteca `maude`, el preludio de Maude y la biblioteca `libmaude.so`. Yo lo tengo en `~/my_python_libs/maude`, así que hago:
   * `$ export PYTHONPATH=$PYTHONPATH:~/my_python_libs`
   * `$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/my_python_libs/maude`
 
   Por comodidad, lo he metido en un _script_ `set_env.sh` y lo cargo con `source set_env.sh`

## Compilación
1. `source /opt/ros/dashing/setup.bash`
1. `colcon build --symlink-install`

## Ejecución
1. `. install/setup.bash`
2. `source set_env.sh`
3. Lanzar el _publisher_ con `ros2 run pubsub_maude_ros mauder_talker`
4. Lanzar el _subscriber_ en otro terminal con `ros2 run pubsub_maude_ros mauder_listener` (se debe ejecutar el paso 1)
