# ROS_SmartPAD

Desarrollado usando Ubuntu 20.04 y ROS Noetic.

Despues de clonar el repositorio, compilelo (catkin_make). No olvide actualizar la terminal para que pueda ver los packages compilados (source devel/setup.bash)
1. corra el demo de RVIZ de algun robot de Moveit.
2. Ahora corra ROS SmartPAD con:
```
roslaunch smartpad smartpad.launch group:=kuka6
```
El parametro group corresponde al nombre del "PLanning Group" de Moveit.
