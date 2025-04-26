# ROS Multi-Robot Simulation

## Descrizione
  
Questo progetto simula uno scenario multi-robot utilizzando **ROS Noetic** e **Gazebo 11.11.0**. Due robot collaborano:
- uno effettua la ricerca attiva di un oggetto rosso;
- l'altro raggiunge l'oggetto una volta individuato.

Sono disponibili tre scenari di simulazione differenti, implementati tramite Gazebo, per testare il comportamento dei robot.

## Installazione

  Per utilizzare questo pacchetto è necessario installare un workspace ROS:
  
  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin_make
  ```
    
  Successivamente va attivato il workspace, operazione che va fatta ogni volta che lo si apre:
    
  ```bash
  source ~/catkin_ws/devel/setup.bash
  ```
    
  Oppure, in modo tale da non dover ripetere l'operazione ogni volta:

  ```bash
  gedit ~/.bashrc
  ```
    
  Quando il file si è aperto, inserire nell'ultima riga: 
  
  ```bash
  source ~/catkin_ws/devel/setup.bash
  ```

  Quindi salvare, chiudere l'editor, e aggiornare il file:
  
  ```bash
  source ~/.bashrc
  ```

  Per installare il pacchetto e attivarlo:

  ```bash
  cd src
  
  git clone <https://github.com/piccioliaurora/ros-multi-robot-simulation>

  cd ..

  catkin_make
  ```

## Utilizzo

  Per prima cosa va lanciata la simulazione Gazebo in un terminale, mentre in un altro terminale viene lanciato il programma relativo ai due robot.
  In ciascuno dei due terminali:
  
  ```bash
  cd ~/catkin_ws
  catkin_make
  ```

  Nel primo terminale si lancia la simulazione Gazebo:

  ```bash
  roslaunch my_robot spawn_world.launch
  ```
    
  (Per lanciare la simulazione relativa al secondo e terzo scenario è equivalente, cambia il nome di file di lancio, 'spawn_world2.launch' e 'spawn_world3.launch' rispettivamente)

  Nel secondo teminale si lancia il programma dei due robot:

  ```bash
  roslaunch my_robot start_py.launch
  ```
    
  
