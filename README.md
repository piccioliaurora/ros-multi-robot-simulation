Descrizione:
  Lo scopo di questo pacchetto è di simulare uno scenario in cui vengono implementati due robot, uno per la ricerca attiva di un oggetto rosso, l'altro per raggiungerlo.
  Sono presenti tre scenari Gazebo per testare il comportamento dei robot.
  Vengono utilizzati ROS Noetic e Gazebo 11.11.00.

Installazione:
  Per utilizzare questo pacchetto è necessario installare un workspace ROS:
    $ mkdir catkin_ws
    $ cd catkin_ws
    $ mkdir
    
  Successivamente va attivatto il workspace, operazione che va fatta ogni volta che si apre il workspace:
    $ source ~/catkin_ws/devel/setup.bash
    
  Oppure, in modo tale da non dover ripetere l'operazione ogni volta:
    $  gedit ~/.bashrc
    
  Quando il file si è aperto. inserire, nell'ultima riga: 
    $ source ~/catkin_ws/devel/setup.bash 

  Per installare il pacchetto, copiare l'url:
    $ git clone <url_del_repository_github>
    
  Compilare il pacchetto:
    $ cd catkin_ws
    $ catkin_make

Utilizzo:
  Per prima cosa va lanciata la simulazione Gazebo in un terminale, mentre in un altro terminare viene lanciato il programma relativo ai due robot.
  In ciascuno dei due terminali:
    $ cd catkin_make

  Nel primo terminale si lancia la simulazione Gazebo:
    $ roslaunch my_robot spawn_world.launch
  (Per lanciare la simulazione relativa al secondo e terzo scenaio è equivalente, cambia il nome di file di lancio, 'spawn_world2.launch' e 'spawn_world3.launch' rispettivamente)

  Nel secondo teminale si lancia il programma dei due robot:
    $ roslaunch my_robot start_py.launch
    
  
