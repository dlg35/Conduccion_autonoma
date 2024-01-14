## Guia de ejecución en simulacion
1. Descargar repositorio.
```
git clone https://github.com/dlg35/Conduccion_autonoma.git
```
2. Acceder a la carpeta y compilar el entorno ROS mediante el comando.
```
catkin_make
```
3. Actualizamos el entorno de ROS.
```
source devel/setup.bash
```
4. Para ejecutar las herramientas de `rviz` y `stage` tendremos que ejecutar el siguiente comando: 
```
roslaunch navigation_stage mi_navigation.launch
```
5. Por último, ejecutar los archivos de detección de colores y el archivo del código base, se accederá a la carpeta en la que están situados `cd src/navigation_stage/src` y ejecutaremos el comando.
```
python3 color_detector.py
```
```
python3 proyecto.py
```

## Guia de ejecución en robot real
1. Modificar el código de los archivos de detección de colores y del código base, cambiando los nombres de los `topics` por los nombres de los topics del robot real.

2. Ejecutar los archivos de detección de colores y el archivo del código base, se accederá a la carpeta en la que están situados `cd src/navigation_stage/src` y ejecutaremos el comando.
```
python3 color_detector.py
```
```
python3 proyecto.py
```




