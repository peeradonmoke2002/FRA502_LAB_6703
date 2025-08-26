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

2. Run submodule command to Add turtlesim_plus package
   ```bash
   git submodule update --init --recursive
   ```
3. Build the workspace
   ```bash
   colcon build --symlink-install
   ```
4. Source the workspace
    in every new terminal, run

   ```bash
   source install/setup.bash
   ```
   However, you can add this line to your ~/.bashrc file to source the workspace automatically when you open a new terminal.
   ```bash
   echo "source /path/to/your/workspace/install/setup.bash" >> ~/.bashrc
   ```
