# FRA502-LAB-6703
Peeradon Ruengkaew 6703 (Moke)



## Insallation

1. Clone LAB repository at barch LAB2
   ```bash
   git clone https://github.com/peeradonmoke2002/FRA502_LAB_6703.git -b LAB2
   ```
   cd into the workspace
   ```bash
   cd FRA502_LAB_6703
   ```

2. Run submodule command to Add turtlesim_plus package and pygame python library
   ```bash
   git submodule update --init --recursive
   ```
   and install pygame: 
   ```bash
   pip install pygame==2.3.0
   ```
4. Build the workspace
   ```bash
   colcon build --symlink-install
   ```
5. Source the workspace
    in every new terminal, run

   ```bash
   source install/setup.bash
   ```
   However, you can add this line to your ~/.bashrc file to source the workspace automatically when you open a new terminal.
   ```bash
   echo "source ~/FRA502_LAB_6703/install/setup.bash" >> ~/.bashrc
   ```
   and source the ~/.bashrc file
   ```bash
   source ~/.bashrc
   ```
6. chmod +x the python files in lab2 and turtlesim_plus packages
   ```bash
   chmod +x src/lab2/scripts/*.py
   ```
   and 
    ```bash
    chmod +x src/turtlesim_plus/turtlesim_plus/scripts/*.py
    ```
    then build and source the workspace again
    ```bash
    colcon build && source install/setup.bash
    ```


## Run the nodes
1. Open a terminal and run the turtlesim node
   ```bash
   ros2 run turtlesim_plus turtlesim_plus_node.py
    ```

2. Spawn second turtle (turtle2)
   ```bash
    ros2 service call /spawn_turtle turtlesim/srv/Spawn "x: 0.0
    y: 0.0
    theta: 0.0
    name: 'turtle2'" 
   ```

3. Open a new terminal and run the eater node
    ```bash
    ros2 run lab2 eater.py
    ```
4. Open a new terminal and run the killer node
    ```bash
    ros2 run lab2 killer.py
    ```
5. Open a new terminal and run the turtlesim_pose node
    ```bash
    ros2 run lab2 turtlesim_pose.py
    ```
6. Open a new terminal and run RViz2 with the provided configuration file

    ```bash
    cd ~/path/to/your/workspace/FRA502_LAB_6703/
    rviz2 -d src/lab2.rviz
    ```

## Usage

1. In the turtlesim window, you will see two turtles: turtle1 (the eater) and turtle2 (the killer).
2. The eater turtle (turtle1) will move around the screen and "eat" base from your clicks on gui and set goal pose in rviz2. It will eat maximum 20 pizzas and pizza will not spawn and count however the function click and set goal to move is still work.
3. After turtle1 has eaten 20 pizzas, turtle2 (the killer) will start chasing turtle1. So try to click or set goal pose to move turtle1 to avoid being caught by turtle2.
4. Can see all action in rviz2
