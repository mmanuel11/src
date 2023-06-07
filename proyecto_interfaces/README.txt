****Instrucciones generales

1. Cree una carpeta con el nombre: 'taller1_grupo5'.
2. Cree una carpeta src dentro de 'taller1_grupo5'.
3. Alli situe el paquete 'turtle_bot_5' descomprimido.
4. Desde el root del 'taller1_grupo5' ejecute el comando 'colcon build'.
5. Ejecute el comando . install/setup.bash.
6. Haga la prueba del funcionamiento del paquete.


**** Dependencias 

Ejecute los comandos:
1. sudo apt install python3-rosdep2
2. rosdep update
3. rosdep install -i --from-path src --rosdistro humble -y


****Ejecutar nodo turtle_bot_interface
ros2 run turtle_bot_5 turtle_bot_interface.py


****Ejecutar nodo turtle_bot_interface
ros2 run turtle_bot_5 turtle_bot_teleop


****Ejecutar nodo turtle_bot_player
ros2 run turtle_bot_5 turtle_bot_player.py



