<p align="center"><img src="./img/ros_logo.png" height="150" alt=" " /></p>
<h1 align="center">ROS Guide</h1> 
<h4 align="right">Jun 23</h4>

![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)

![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)

<br>


- [Conceptos](#conceptos)
  - [Roscore](#roscore)
- [Workspace](#workspace)
  - [Catkin](#catkin)
  - [Archivo .bashrc](#archivo-bashrc)
    - [Red ROS](#red-ros)
  - [Achivo CmakeList.txt](#achivo-cmakelisttxt)
  - [Archivo package.xml](#archivo-packagexml)
- [Herramientas](#herramientas)
  - [Rviz](#rviz)
  - [Gazebo](#gazebo)
- [Pila de navegación de ROS](#pila-de-navegación-de-ros)
  - [Odometría](#odometría)
  - [Algoritmo de localización adaptativo de Monte Carlo (AMCL)](#algoritmo-de-localización-adaptativo-de-monte-carlo-amcl)
  - [Transformadas de los sensores](#transformadas-de-los-sensores)
  - [Funcionamiento de la pila](#funcionamiento-de-la-pila)
  - [Mapas de costo](#mapas-de-costo)
  - [Planificadores](#planificadores)
  - [Recovery Behaviors](#recovery-behaviors)
  - [Parámetros](#parámetros)
- [Puesta en marcha de un robot Turtlebot](#puesta-en-marcha-de-un-robot-turtlebot)
  - [Esto Funciona para conectar 2 computadores con ROS1](#esto-funciona-para-conectar-2-computadores-con-ros1)
  - [Comunicación entre ordenadores](#comunicación-entre-ordenadores)
  - [Conectar ROS Máster](#conectar-ros-máster)
  - [Espacio de trabajo](#espacio-de-trabajo)
  - [Preparar Turtlebot](#preparar-turtlebot)
  - [Teleoperación por teclado](#teleoperación-por-teclado)
  - [Verificar el funcionamiento de la cámara](#verificar-el-funcionamiento-de-la-cámara)
  - [Construir un mapa (SLAM)](#construir-un-mapa-slam)
  - [Navegación autónoma en un mapa conocido](#navegación-autónoma-en-un-mapa-conocido)
  - [Aparcamiento automático](#aparcamiento-automático)

## Robot Operating System (ROS)
Entorno de desarrollo utilizado para el desarrollo de aplicaciones robóticas. Es de código abierto,cuyo código fuente puede ser estudiado, modificado y utilizado libremente con cualquier finalidad. Proporciona librerías y herramientas para el desarrollo de programas robóticos. No es un sistema operativo, sino que es un marco de trabajo o “framework”.

## Paquetes
Todo el software de ROS está organizado en paquetes. Un paquete es una colección coherente de ficheros (ejecutables y ficheros de soporte) que sirven para un propósito específico.

Estos paquetes son definidos por un manifiesto `fichero package.xml` que da información sobre el paquete, incluyendo su nombre, versión, descripción, información de licencia, dependencias.

El directorio que contiene el paquete se llama `package directory` y no puede haber más de un paquete por directorio.

El paquete contiene los procesos de ejecución `nodos`, librerías, scripts,
archivos de configuración `Makefiles`, etc.

> :memo: **Note:** Para crear un paquete debes navegar primero hasta el
directorio src de tu workspace
```
cd ~/catkin_ws/src
```
`Las dependencias` son otros paquetes y bibliotecas que requiere
nuestro paquete. Se pueden incluir ahora o a posteriori editando
los ficheros de configuración CMakeLists.txt y package.xml

> :memo: **Note:** Todos los paquetes del workspace se compilan juntos:
```
cd ~/catkin_ws/
catkin_make
```

> :memo: **Note:** Alternativamente a la creación de un paquete lo podemos descargar de Github u otro repositorio. Pero seguirá siendo necesario ejecutar el comando **catkin_create_pkg**


## Pilas (Stack)
Las pilas son una colección de paquetes con funcionalidad relacionada.

## Manifiesto del paquete
El manifiesto `package.xml` proporciona información sobre el paquete incluyendo su nombre, versión, descripción, información de licencia, dependencias, paquetes exportados.

## Tipos de mensajes
Los mensajes son datos enviados de un proceso (nodo) a otro.

El directorio msg contiene información con las descripciones de los mensajes en un archivo “.msg”. Este define las estructuras de datos para los mensajes que publican los diversos procesos. En ese archivo hay dos partes: campos y constantes. Los campos son los datos que se envían dentro del mensaje. Las constantes definen valores útiles que se pueden usar para interpretar esos campos.
```
int32 x
int32 y = 123
```
Admite los tipos estándar (integer, floating point, boolean, etc.) pero también se puede crear tipos personalizados.
```
std_msgs/String
std_msgs/Int32
std_msgs/bool
std_msgs/Empty
```

## Servicios
Los nodos también se pueden comunicar a través de un patrón de interacción entre nodos cliente-servidor, apropiado para las interacciones de petición-respuesta.

Utilizan dos mensajes, uno para petición y otro para respuesta.

Un nodo ofrece un servicio y otro nodo lo utiliza enviando una petición y esperando una respuesta.

## Directorio SRC o scripts
Directorio que contienen el código fuente de los programas que utilizará
el paquete.

## Directorio Launch & Launch files
launch contiene ficheros Launch (“.launch”), los cuales normalmente sirven para ejecutar varios nodos a la vez. Son Ficheros XML que tienen extensión .launch.
Para lanzar un launch file:
```
roslaunch package_name launch_file
```
> :bulb: **Tip:** Se recomienda crear una carpeta **launch** para incluir los launch files de nuestro paquete.

## Nodos
Es un ejecutable dentro de un paquete ROS, escrito con la biblioteca de C++ `roscpp` o de Python `rospy`. Cada nodo es un proceso que realiza una tarea.

> :bulb: **Tip:**  Los nodos son los ejecutables de ROS

> Los nodos pueden publicar mensajes en un topic.

> Los nodos pueden subscribirse a los topics para recibir
mensajes. Todos los nodos subscritos a un topic reciben los
mismos mensajes.

>Los mensajes de un topic tienen la misma estructura de datos. Algunas estructuras están predefinidas en ROS, también se pueden crear nuevas estructuras de mensajes.

Para crear un nodo en nuestro paquete, tendremos que editar las
siguientes líneas del fichero ***CmakeLists.txt***:
```
add_executable(${PROJECT_NAME}_node src/my_first_package_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} $
{catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
)
```
Esto nos permitirá crear un nodo llamado ***my_first_package_node***. Podemos adaptar los cambios para crear otros nodos en el paquete. Luego tendremos que crear el fichero ***my_first_package_node.cpp***.


## Máster
El ROS Master permite la comunicación entre los nodos. Sin el ROS Máster, los diferentes nodos del grafo no se podrían encontrar unos a otros, y, como consecuencia, no podrían intercambiar mensajes ni invocar servicios.

Al ejecutar el ROS Máster, también se ejecuta el servidor de parámetros `Parameter Server`, el cual utilizan los nodos para almacenar y recuperar parámetros en tiempo de ejecución.Además, se ejecuta el nodo `rosout`, el cual se comporta como la salida estándar. Estos tres nodos siempre van de la
mano.

## Topics
Los nodos se pueden comunicar a través de un patrón de interacción publicador-suscriptor mediante los mensajes. Para poder comunicarse, se crean los topics, que son canales de información entre nodos. Luego un nodo puede comunicarse con otro publicando mensajes en un topic al cual el otro nodo tiene que estar suscrito (suscriptor).

Un topic puede tener diversos publicadores y suscriptores concurrentes y un nodo puede publicar y/o suscribirse a varios topics.

Los publicadores y los suscriptores no son conscientes de la existencia de los demás.

## Bags
son un formato de almacenamiento de datos de ROS que permiten guardar datos enviados a través de mensajes, suscribiéndose al topic que se requiera y guardando los mensajes que se publiquen en un fichero. Estos ficheros también se pueden volver a reproducir en el mismo topic en el que fueron grabados. Son muy utilizadas para estudiar mensajes que publican los láseres, ya que a tiempo real publican mensajes.

<br>

# Conceptos
## Roscore
Son un conjunto de nodos y programas que son necesarios y por lo tanto habrá que ejecutar en cualquier aplicación de ROS. Roscore va a iniciar tres apartados:

* ROS Master (necesario para que los demás nodos de ROS puedan comunicarse entre ellos.)
* Servidor de parámetros (ROS Param)
* Rosout (Equivalente en ROS a stdout/stderr)

> :memo: **Note:** Un sistema ROS solo puede tener un único nodo máster corriendo

# Workspace
ROS está preparado para trabajar en un área de trabajo (workspace), esta área de trabajo no es más que una carpeta en nuestro ordenador. Esta carpeta es la carpeta en la que vamos a introducir los paquetes necesarios para ejecutar nuestra aplicación. La manera de agregar paquetes a esta carpeta y la forma y estructura que estos paquetes van a tener, viene dada por el sistema ```catkin``` (anteriormente “rosbuild”).

> :bulb: **Tip:** Es posible utilizar varios workspaces
> 
Es posible utilizar varios workspaces, para ello creamos otro:
```
mkdir -p ~/otro_catkin_ws/src
cd ~/otro_catkin_ws/
catkin_make
```
Ahora habría que hacer ***source*** para que cuando abramos una consola nueva sepa que el workspace está en esa carpeta, para ello escribiremos ***source ~/otro_catkin_ws/devel/setup.bash*** al final del archivo `~/.bashrc` de nuestro sistema Linux. Quedando el final de nuestro archivo ~/.bashrc:
```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/otro_catkin_ws/devel/setup.bash
```

```La primera línea indica a Linux donde está instalado ROS, la segunda indica a Linux que active nuestro workspace creado en primer lugar y la segunda indica a Linux que active nuestro workspace creado en segundo lugar desactivando el primero```.

> :bulb: **Tip:** El último workspace al que le hemos hecho source será el que está activado

para cambiar de un workspace a otro basta con hacer “source” del workspace que queremos usar, por ejemplo para volver a activar el primer workspace escribiremos de nuevo:

```
source ~/catkin_ws/devel/setup.bash
```
esto activará el primer workspace y desactivara el segundo

## Catkin
no es más que una plantilla para hacer nuestros programas y paquetes, de tal manera que estos van a ser fácilmente interpretados por cualquier programa facilitando la portabilidad de estos.En definitiva, si queremos hacer una aplicación robótica con ROS, tanto el workspace como cada paquete dentro del workspace, deberán estar hechos bajo el sistema catkin.

> :memo: **Note:** El nuevo sistema de compilado para ROS es "catkin", mientras que "rosbuild" lo es para los sistemas ROS viejos.

## Archivo .bashrc
### Red ROS
Se puede configurar el .bashrc de cada ordenador en la red LAN para
que el máster se ejecute en un ordenador y los otros ordenadores se
conecten a él.

http://wiki.ros.org/ROS/Tutorials/MultipleMachines

## Achivo CmakeList.txt
Es como la receta, es una plantilla, de tal manera que, si otro paquete u otro workspace quiere usar nuestro workspace o un paquete contenido en este, este archivo le ayudará a comprender la información.

```Catkin``` se usa para poder construir software, para ello se necesita información como puede ser donde se encuentra el código fuente, donde están las librerías, dependencias, etc.. Toda esta información se encuentra en el archivo ```CmakeLists.txt```

El fichero CMakeLists.txt de nuestro paquete contiene información acerca de la forma de compilar nuestro paquete, habrá que editarlo cada vez que:

* Se cree un nuevo nodo en el paquete
* Se creen mensajes, servicios o bibliotecas
* Añadamos dependencias, etc.


## Archivo package.xml
se encargará de definir el nombre del paquete, el número de versión de este, autores y lo más importante, dependencias.

El fichero ```package.xml``` contiene metadatos tales como número de versión de nuestro paquete, licencia de uso, autor e información sobre dependencias. Habrá que editarlo para actualizar los metadatos y, especialmente, cuando se añadan
dependencias.





<br>
<br>








# Herramientas 

## Rviz
Herramienta de visualización de ROS que puede mostrar una gran variedad de información, como puede ser el mapa de la sala, la
trayectoria que va a realizar el robot o la imagen que transmite la cámara a tiempo real. También es capaz de mostrar una representación del robot utilizado.

Además de permitir visualizar gran información del sistema, también permite publicar mensajes sobre diversos topics.

## Gazebo
Entorno de simulación de ROS, Permite simular comportamientos del robot como si fuera real. Es de gran utilidad a la hora de probar programas antes de meterlos en el robot real, evitando que pueda romperse debido a algún fallo de programa.

<br>

# Pila de navegación de ROS
Va a ser la base de la generación de trayectorias y del sistema de navegación autónomo del robot. toma información de la odometría y de los sensores que recopilan datos sobre el entorno. Con esto, es capaz de estimar la posición del robot dentro de un mapa previamente generado, mediante el algoritmo de localización adaptativo de Monte Carlo (AMCL 2) y generar trayectorias y modificarlas en función de los objetos que se encuentre por el camino y que no estén en el mapa (por ejemplo, una persona andando).

## Odometría
La odometría es el estudio de la estimación de la posición de vehículos con ruedas durante la navegación. Esta estimación se basa en la rotación de las ruedas para calcular los cambios de posición a lo largo del tiempo.

## Algoritmo de localización adaptativo de Monte Carlo (AMCL)
El algoritmo de localización de Monte Carlo se basa en un filtro de partículas para que los robots puedan localizarse dentro de un entorno dado. Este filtro de partículas representa la distribución de estados posibles, es decir, cada partícula representa una hipótesis de dónde se encuentra el robot. Cada vez que el robot se mueve o detecta algo, las partículas se desplazan intentando predecir su nuevo estado, viendo si es coherente el estado que han previsto con el real obtenido por los sensores. Al final, las partículas deberían converger hacia la posición real del robot. Para esto, trabaja en paralelo con la odometría.

## Transformadas de los sensores
Debido a la utilización de la odometría y de la información recibida de los sensores, es necesario que el sistema sepa en que posición en el cuerpo del robot se encuentran las ruedas o los láseres.

Para ello se utilizan diferentes sistemas de referencia para cada elemento. Se establece un sistema de referencia “padre” (suele ser el centro de la base), y a partir de él se establecen los demás. Para definir la relación entre el sistema de referencia “padre” con cualquier otro se utilizan las transformadas.

## Funcionamiento de la pila
Tras haber explicado las bases para la localización de un robot dentro de un
mapa, podemos explicar la generación de trayectorias que ofrece la pila
explicando varios conceptos importantes:
* Local_costmap
* Global_costmap
* Local_planner
* Global_planner
* Recovery_behaviors

## Mapas de costo
Para la generación de las trayectorias, la pila utiliza dos mapas. Con el mapa que se le suministra al paquete (fichero .yaml), el sistema crea un mapa de costo denominado “mapa global” (global_costmap), utilizado para la posición general. Además, crea otro mapa de costo llamado “mapa local” (local_costmap) que supone una reconstrucción del mapa a corta distancia, generado por la lectura de los sensores, posicionando obstáculos si los hay.

A medida que se encuentran nuevos obstáculos, el mapa de costo global (global_costmap) es actualizado por el sistema con estos obstáculos, pero no reescribe el fichero .yaml de dónde se obtiene este mapa. Esto hace que obstáculos imprevistos puedan ser borrados sin necesidad de cambiar el mapa. Por ejemplo, si el mapa está guardado con las puertas abiertas de una sala y de repente se encuentra el obstáculo de que una puerta está cerrada, 38 esto haría que el sistema actualizase el mapa de costo global añadiendo un obstáculo ahí, pero no modificaría el mapa. Además, al abrir la puerta este obstáculo se borraría por sí solo.

## Planificadores
El sistema de navegación va a estar pendiente todo el rato al topic “move_base_simple/goal”, donde se publicarán las coordenadas del punto final al que se quiere llegar dentro del mapa. Cuando se recibe un mensaje en este topic, el sistema crea una primera trayectoria global a seguir a largo plazo con los datos recibidos del mapa de costo global (global_planner).

Para poder sortear obstáculos que no están contemplados en el mapa de costo global, se toman los datos del mapa de costo local y, con ellos, el sistema intenta encontrar una trayectoria a seguir de manera inmediata (trayectoria local) para sortear el obstáculo (local_planner, encargado de publicar los comandos de velocidad). Para que el robot se mueva, es necesario tener ambas trayectorias.

## Recovery Behaviors
El sistema cuenta con mecanismos de recuperación si el robot no consigue llegar a la meta establecida.

Si no se encuentra trayectoria posible debido a que hay obstáculos de por medio, el robot se para en una posición. En esa posición, el robot resetea los mapas de costo a partir de un radio dado desde el centro del robot (3 metros por defecto, pero se puede cambiar en el parámetro “conservative_reset”), borrando los obstáculos que pueda haber fuera de ese radio. Si sigue atrapado, da una vuelta de 360º actualizando los mapas de costo con lo que vea, comprobando si tiene salida por algún otro lado. Si no la encuentra, reinicia los mapas de costo de manera más agresiva, borrando todos los obstáculos en un radio menor que el anterior (1,84 metros si no se ha cambiado el parámetro “aggressive_reset”), y, si con esto sigue igual, da otra vuelta de 360º. Tras esta, si no encuentra ninguna salida, el robot aborta el plan debido a que no se puede llegar a la meta. En la ilustración 31 se puede observar un gráfico con los diferentes estados del mecanismo de recuperación.

## Parámetros
Para optimizar la navegación autónoma, los diferentes mapas y planificadores tienen diversos parámetros que se pueden editar en función de lo que se requiera. Como parámetros importantes se pueden encontrar la distancia mínima que puede haber entre el robot y los diferentes obstáculos (inflation_radius), la dimensión del mapa local o los límites de velocidad y aceleración del robot.

<br>

# Puesta en marcha de un robot Turtlebot 
## Esto Funciona para conectar 2 computadores con ROS1
En este apartado se describen los pasos a realizar para la puesta en marcha del robot Turtlebot. Cómo establecer la comunicación entre el PC controlador del robot con otro ordenador y cómo hacer que el robot empiece a funcionar. Además, se describirán paquetes básicos que se van a utilizar.

## Comunicación entre ordenadores
Al no poder tener una pantalla conectada al mini ordenador controlador del robot mientras está en movimiento, se necesita que éste se pueda manejar desde otro ordenador. Para ello se utiliza el protocolo SSH (Secure Shell), el cual utiliza una arquitectura cliente/servidor y permite al usuario conectarse a un host remotamente por medio de un canal seguro en el que toda la información está cifrada, siempre y cuando esté Para ello, primeramente, hay que tener instalado el servidor SSH en el
ordenador de la Turlebot. Tras esto, ya se puede acceder vía SSH al ordenador.

Para ello en la terminal se utilizará el siguiente comando:
```
> ssh turtle@<TURTLEBOTP_IP>
```
Donde “turtle” es el nombre de usuario al que se quiere entrar y ```<TURTLEBOTP_IP>``` es la dirección IP de la Turtlebot. Tras ponerlo, pedirá la contraseña y se abrirá la terminal del PC del robot. No se tiene acceso a la visualización del escritorio, sino solo a su consola. Por cada terminal que se
requiera, se necesitará ejecutar el comando SSH en otra terminal del ordenador fijo.

## Conectar ROS Máster
Tras conseguir utilizar el ordenador de la Turtlebot de manera remota, ahora lo importante es conseguir que los nodos lanzados desde un ordenador u otro se puedan comunicar entre ellos. Para esto se necesita que ambos ordenadores puedan encontrar el ROS Máster y que sea el mismo. Para ello, hay que utilizar los siguientes comandos.

En el ordenador de la Turtlebot se exportan las variables ROS_MASTER_URI Y ROS_HOSTNAME con los siguientes valores:
```
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=IP_OF_TURTLEBOT
```
Donde localhost e IP_OF_TURTLEBOT son la dirección IP del ordenador de la Turtlebot.

Mientras que en el ordenador fijo hay que exportar estas variables con los
siguientes valores:
```
export ROS_MASTER_URI=http://IP_OF_TURTLEBOT:11311
export ROS_HOSTNAME=IP_OF_PC
```
Donde IP_OF_TURTLEBOT es la dirección IP del ordenador de la Turtlebot e IP_OF_PC es la dirección IP del ordenador fijo.

```Estos comandos habría que escribirlos cada vez que se abra una nueva terminal. Para evitar esto, se escribirá en el /.bashrc de cada ordenador, sus respectivos comandos. Esto se debe a que el archivo /.bashrc es un script que se ejecuta cada vez que se inicia una nueva terminal, ejecutando todos los comandos que tenga en su interior. Luego si se le añaden estos comandos, se ejecutarán nada más abrir la terminal, evitando que se requiera ponerlos cada vez que necesitamos una nueva terminal.```

Tras esto, para comprobar que ambos ordenadores están conectados al mismo ROS Máster, se lanza roscore y se mira la lista de topics de ambos ordenadores, donde deberían aparecer los mismos topics en los dos.

## Espacio de trabajo
Lo más cómodo a la hora de trabajar con ROS es crear un espacio de trabajo, es decir, un directorio donde vayan a estar guardados todos los paquetes que se hayan realizado para el proyecto. ROS guarda por defecto los paquetes instalados en la ruta /opt/ros/kinetic/. `Se podrían guardar aquí los paquetes realizados, pero para modificar documentos de este directorio son necesarios privilegios de súper usuario, además de que queda tener directorio propio para los paquetes del proyecto hace que esté mejor organizado.`

`ROS posee una herramienta denominada catkin, la cual se utiliza para construir paquetes. Para poder tilizarlo es recomendado tener un espacio de trabajo.` Esta herramienta permite crear paquetes de manera más cómoda que haciendo todo a mano, creando los ficheros necesarios con las dependencias que requiera el paquete.

En el proyecto se ha creado un espacio de trabajo llamado “catkin_ws”, en él se crea una carpeta denominada “src” donde se guardan los paquetes creados y/o copiados de la comunidad de ROS para el proyecto.

`Siempre que se vayan a utilizar nodos de algún directorio, hay que compilar el archivo de configuración del entorno. Para ello, cada vez que se utilice una nueva terminal, se escribirán los siguientes comandos:`
```
$ source /opt/ros/%YOUR_ROS_DISTRO%/setup.bash
$ source /root/catkin_ws/devel/setup.bash
```
Siendo %YOUR_ROS_DISTRO% el nombre de la distribución instalada de ROS (por ejemplo, kinetic), ya que en esta ruta se guardan todos los nodos que han sido instalados.

El segundo comando compila los archivos de configuración del espacio de trabajo.

`Para no tener que escribir estos comandos cada vez que se inicie una nueva terminal, se pueden escribir estos comandos en el archivo /.bashrc, como en el caso anterior.`

## Preparar Turtlebot
Una vez instalados todos los paquetes necesarios, en especial de la pila “Turtlebot”; habiendo realizado la comunicación entre ambos ordenadores y compilado los archivos de configuración del entorno, es posible comenzar a manejar el robot.

Para el inicio del Robot, existe un paquete denominado “Turtlebot_bringup”. Para poder ejecutar este Launch se escribirá el siguiente comando:
```
$ roslaunch turtlebot_bringup minimal_with_hokuyo.launch
```
## Teleoperación por teclado
El movimiento del robot puede ser controlado por teclado, ya sea vía SSH o desde el PC fijo (si la comunicación se ha configurado bien), siempre y cuando se haya arrancado la Turtlebot como se ha explicado en el apartado anterior.

Para ello, la pila “Turtlebot” contiene un paquete denominado “turtlebot_teleop” con el script “turtlebot_teleop_key”, que puede ser ejecutado como nodo con el siguiente comando:
```
$ roslaunch turtlebot_teleop keyboard_teleop.launch
```

## Verificar el funcionamiento de la cámara
```
$ roslaunch turtlebot_bringup 3dsensor.launch 3dsensor:=astra
$ rosrun image_view image_view image:=/camera/rgb/image_raw
```

## Construir un mapa (SLAM)
El paquete “turtlebot_navigation” proporciona las herramientas necesarias para crear un mapa a partir de los datos recibidos del entorno por medio de los sensores y la odometría.

Un algoritmo SLAM (del inglés, “Simultaneous Localization And Mapping”) es aquel que consigue que el robot construya un mapa del entorno y que se localice mientras navega en él de manera simultánea. Esto se consigue gracias al archivo Launch del paquete “turtlebot_navigation” llamado “gmapping_demo.launch”, el cual ejecuta un algoritmo SLAM.

Para poder utilizarlo se requiere que el robot proporcione datos de odometría y que tenga un telémetro láser fijo y montado horizontalmente, pudiendo ser la cámara ya que, como se ha explicado en el apartado, puede simular el escáner de un láser escribiendo en el topic /scan.

Este algoritmo SLAM ejecutado es el que proporciona el paquete “gmapping” proporcionado por ROS, denominado “slam_gmapping”.

## Navegación autónoma en un mapa conocido

Una vez generado el mapa, se podrá realizar una navegación autónoma por el entorno mapeado. Para ello, tras arrancar la Turtlebot, hay que ejecutar el archivo Launch “amcl_demo.launch” del paquete  turtlebot_navigation”, al cual se le ha de pasar el mapa creado:
```
$ roslaunch turtlebot_navigation amcl_demo.launch map_file:=/ruta/my_map.yaml
```

## Aparcamiento automático
El robot puede llegar automáticamente a la estación de carga gracias a rayos infrarrojos. La estación de carga emite luces infrarrojas cubriendo tres regiones frente a ella, izquierda, centro y derecha, cada una dividida en dos campos: cerca y lejos. Cada haz codifica esta información, luego el robot al recibirlo sabe en qué región se encuentra.

<br>

Link: http://wiki.ros.org/

<br>

---
Copyright &copy; 2022 [carjavi](https://github.com/carjavi). <br>
```www.instintodigital.net``` <br>
carjavi@hotmail.com <br>
<p align="center">
    <a href="https://instintodigital.net/" target="_blank"><img src="./img/developer.png" height="100" alt="www.instintodigital.net"></a>
</p>