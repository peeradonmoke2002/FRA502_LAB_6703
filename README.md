# FRA502-LAB-6703
Peeradon Ruengkaew 6703 (Moke)

## Table of Contents

- [Installation](#insallation)
- [Run the launch file](#run-the-launch-file)

## Insallation

1. Clone LAB repository at barch LAB2
   ```bash
   git clone https://github.com/peeradonmoke2002/FRA502_LAB_6703.git -b LAB3
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


## Run the launch file

```bash
ros2 launch lab3 lab3_bringup.launch.py 
```