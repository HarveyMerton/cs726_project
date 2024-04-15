COMPSYS726 Practical Project 

Author: 
Harvey Merton 
hmer101 
979936166 

To run the solution, follow these steps: 
1. Launch the simulation world on Gazebo. 
    - In a new terminal window, navigate to the folder where the simulation world is stored
    - Then run the following command. Note that this is an example which launches a world called 'example_2.world', please replace this with the name of the simulation world.   

        roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$PWD/example_2.world


2. Setup catkin workspace using the following steps: 
- Open a new terminal window
- Navigate to the /home/<USR> directory where <USR> is replaced by the name of the 
current user on your machine (this will look like just a '~' on terminal).

Run the following commands to create the directory structure: 
- mkdir -p compsys726/src
- cd ~/compsys726
- catkin_make
- cd src
- catkin_create_pkg python_nodes rospy std_msgs
- cd python_nodes/src


Move the files from Solution_hmer101 to the correct locations:
- The files provided in Solution_hmer101 are in a folder structure that mirrors the structure just created. 
- Copy all files in the Solution_hmer101 from their folders into the corresponding folders
created in the first half of this step. 

Files/folders to copy: 
- src/python_nodes: launch/python.launch (copy folder with file in it)

- src/python_nodes/src: Obstacle_pictures, bumper.py, imu.py, laserscan.py, obstacle_avoid.py, takephoto.py


Use ls on the src/python_nodes/src directory while in the terminal. Ensure that all 
files just coppied into this folder appear green. If not, they may not yet have been marked 
as executable. To fix this, run the following command: 

chmod +x bumper.py imu.py laserscan.py obstacle_avoid.py takephoto.py


For the remaining steps, work out of the ~/compsys726 folder structure just created


3. Open the takephoto.py file and change the directory to store images in on line 26. 
    - The initial directory is: '/home/harvey/compsys726/src/python_nodes/src/Obstacle_pictures'
    - Change 'harvey' to the name of the current user on your machine

    i.e. '/home/<USER>/compsys726/src/python_nodes/src/Obstacle_pictures'


4. Source the launch file 
    - In a new terminal, run the following command:
        source ~/compsys726/devel/setup.bash
    
    If launching from another directory, the full path will have to be given like:
    e.g. source '/home/harvey/Downloads/Solution_hmer101/devel/setup.bash'

5. Run the launch file
    - Use the command: roslaunch python_nodes python.launch

    - If this command fails, it may be due to other Student's files using the same 
        package names (as this is the default one from the lab). In this case, use the 
        following command which more explicitly selects the launch file: 

        roslaunch ~/compsys726/src/python_nodes/launch/python.launch


N.B. Should the simulation have to be restarted, Gazebo must be closed and re-opened i.e. the simple 'reset world' command will not work. This is because the /odom topic that this solution relies on is not reset on resetting the world.


This process was checked many times to ensure that it worked. If you have any issues, please contact me on: hmer101@aucklanduni.ac.nz


