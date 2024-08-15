## UTILS PKG

This package contains useful class, methods and functions about: 

1. ROS Visualization
2. Point cloud library utilities (tabletop segmentation, HSV conversion fix, subsampler [random and classic] ..etc )
3. Eigen to ROS template, matrices operations, etc.
4. io utilities
5. Nearest neighbour algorithm. 
6. Simulation on V-Rep / CoppeliaSIM : Vrep converter convert image depth and rgbd to PointCloud, using CoppeliaSIM or also v-rep old plugin

Converter for PCL formats : 
  PCL_INFO ("Available formats types for SOURCE and DEST:\n"
           "\tOBJ (Wavefront)\n"
           "\tPCD (Point Cloud Library)\n"
           "\tPLY (Polygon File Format)\n"
           "\tSTL (STereoLithography)\n"
           "\tVTK (The Visualization Toolkit)\n\n");

usage : rosrun util_pkg converter_pcl /scene.pcd /scene.obj



## HOW TO GLOBAL FRAME REGISTRATION in simulation
Download CoppeliaSIM:
[download](https://www.coppeliarobotics.com/downloads)


Questo launch file carica il nodo global_frame_registration, quello dell'articolo. 
Con load_reference settato a false ci permette di registrare i frame. All'interno del launch file è possibile settare i parametri o copiare 
la calibrazione fatta. 
tool_subscribers: [/pointPose]
> roslaunch util_pkg coppeliaGlobalFrame.launch load_reference:=false 
Quando si avvia per calibrare chiede di prendere N punti + 1, settati all'iterno del file. Per esempio prendo i 3 punti all'interno della circonferenza, più il punto sopra la board per settare la normale

```
[ INFO] [1591001595.784838678]: Reading position from topic: /pointPose
[ INFO] [1591001595.785240552]: Position your arm to position 1 of 3 and press 'Enter'
```


> roslaunch util_pkg coppeliaGlobalMarker.launch
avvia il riconoscimento del marker e la scena di rviz per visualizzare il punto

il topic pointPose è quello pubblicato dallo script aruco_global_frame che si trova dentro la cartella scripts di util pkg

> aruco_global_frame.py (da chiudere e rilanciare ogni volta che voglio prendere un punto)
all'interno del file sono da "tunare" i paremetri
        self.radius = 0.048 
e
            pose_transformed.pose.position.z = pose_transformed.pose.position.z # + (self.radius * math.cos(270 * math.pi/180))
            pose_transformed.pose.position.x = pose_transformed.pose.position.x # + (self.radius * math.sin(270 * math.pi/180))
            pose_transformed.pose.position.y = pose_transformed.pose.position.y +   0.005             + 0.04   

Nel caso del primo punto, di norma il radius lo metto 0.046, decommento la z e la x e l'angolo di riferimento è 270, tolgo l'offset lungo la y , che quindi rimane 0.005
Nel secondo punto  rimetto radius a 0.048, angolo 150 (a volte nel range 145-150)
Nel terzo punto  rimetto radius a 0.048, angolo 30 (a volte nel range 30-35)

PER LA SCENA NON C'È OFFSET SULLA y (quindi ho commentato il 0.005) CREATA I VALORI SONO:
- per il primo punto angolo: 270
- per il secondo punto angolo : 147
- per il terzo punto angolo : 33, andrebbe però aumentato il radius di 1mm (nel test fatto non ho modificato il radius)
- per il quarto punto, commento come nell'esempio sopra citato la somma parametrica sul radius, e devo prendere un punto sopra la board, quindi decommento l'offset

I parametri variano a seconda dell'inclinazione della camera, dato che non usiamo l'immagine rettificata per riconoscere il marker, non tiene conto della distorsione. Per questo conviene tunare in base alla visualizzazione su rviz !

Una volta terminata la calibrazione, questa viene scritta nel parameter_server di ros. 

su un altro terminale si può fare il comando 
```
 rosparam get /global_frame_registration/
```
e copiarla nel launch file coppeliaGlobalFrame , per ri utilizzarla con load_reference:=true (attenzione alla sintassi / indentazione xml che può causare problemi e copiare fino a position!)

[1 GIUGNO 2020] per adesso abbiamo solo un subscriber pointPose, quindi il global_frame darà un errore perché si aspetta almeno 2 subscriber.
```
[ERROR] [1591002501.918461080]: Error transforming pose 'Pose' from frame 'camera_color_optical_frame' to frame 'world'
```
Aggiungerò un robot / o altre pose nel simulatore per finalizzare la calibrazione, nello step successivo


### Who do I talk to? ###

* Andrea Roberti
* email : andrea.roberti@outlook.com 
* email : andrea.roberti@univr.it
