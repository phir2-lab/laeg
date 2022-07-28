# Instructions

Following this guide, you will run the loop exploration approach based on the Loop-aware Exploration Graph (see the [article](https://doi.org/10.1016/j.robot.2022.104179)). This implementation uses a simulated robot by MobileSim and a [modified version](https://github.com/phir2-lab/gmapping_with_visited_region) of [GMapping SLAM](https://openslam-org.github.io/gmapping.html).

If you use the LEAG, the Loop Exploration, or any map form the maps folder in an academic work, please cite the original [article](https://doi.org/10.1016/j.robot.2022.104179).

**Note**: This implementation was tested in Ubuntu 20.04.4 LTS

## How to compile

The project is compiled using catkin. From the project root directory:

```
catkin_make
```

## How to run 

The project pipeline is composed of four elements that must be executed in sequence, each one in its own terminal.

### Run MobileSim:

The robot is a Pionner Pioneer 3-DX equipped with a SICK LMS 200 laser range-finder simulated by the MobileSim. After installing the MobileSim, open it:

```
MobileSim
```

**Note:** You need to have a map to import in MobileSim; some samples are provided in the maps folder.

You can define the map to be loaded by MobileSim via command line:

```
MobileSim -m maps/castle.map
```

### Run Rosaria:

Rosaria is the bridge between the robot and ROS. From the root directory, run these commands:

```
source devel/setup.bash
roslaunch loop_exploration rosaria.launch
```

### Run GMapping SLAM:

The [GMapping SLAM](https://openslam-org.github.io/gmapping.html) provides the map and robot's pose estimations. The LAEG needs a map with the visited region (see the [article](https://doi.org/10.1016/j.robot.2022.104179) for more details). The version of GMapping modified to provide the visited region is available [here](https://github.com/phir2-lab/gmapping_with_visited_region). After compiling it, run these commands from its root directory:

```
source devel/setup.bash
roslaunch gmapping gmapping.launch
```

### Run Exploration:

Finally, to execute the exploration, run from the LAEG root directory:

```
source devel/setup.bash
./build/loop_exploration/exploration config_file.ini
```

**Note:** You need to have a configuration file, a sample is provided in *src/loop_exploration/configs/config_sample_file.ini*

## Configuration file

The parameters of the configuration file are:

- `slam_system`: Defines the SLAM system to be used. So far, only GMapping is supported. To use it, set *slam_system = GMAPPING*.
- `save_log`: Defines if a log must be saved (*save_log = true*) or not (*save_log = false*)
- `output_address`: Defines the output folder to store the outputs. 
- `planning_iterations_per_second`: Defines the maximum planning interations per second. We suggest to use *planning_iterations_per_second = 4*.
- `map_resolution`: Defines the grid map resolution in meters. We suggest to use *map_resolution = 0.1*.
- `map_width_height`: Defines the fixed maximum width and height of gird map in cells. We suggest to use *map_width_height = 2000*.
- `free_threshold`: Defines the threshold to consider a cell free. We suggest to use *free_threshold = 40*.
- `obstacle_threshold`: Defines the threshold to consider a cell occupied. We suggest to use *obstacle_threshold = 60*.
- `external_border`: Defines the range, in cells, of unknown cells surrounding the map that will be considered as a traversable area. We suggest to use *external_border = 40*.
- `inflate_obstacles_pad_size`: Defines the range of cells surrounding the obstacles that will be considered as obstacles, inflating them. We suggest to use * inflate_obstacles_pad_size = 2*.
- `min_dist_to_obstacles_from_unknown_cells`: Defines the minimum Euclidean distance (in cells) between outer frontiers and obstacles. We suggest to use *min_dist_to_obstacles_from_unknown_cells = 4*.
- `min_dist_to_obstacles_from_unknown_cells`: Defines the minimum Euclidean distance (in cells) between inner frontiers and obstacles. We suggest to use *min_dist_to_obstacles_from_unknown_cells = 2*.
- `max_euclidean_dist_to_merge_frontiers`: Defines the maximum Euclidean distance (in cells) between two outer frontiers to merge them. We suggest to use *max_euclidean_dist_to_merge_frontiers = 10*.
- `max_nodes_dist_to_merge_frontiers`: Defines the maximum distance (in graph nodes) between two outer frontiers to merge them. We suggest to use *max_nodes_dist_to_merge_frontiers = 2*.
- `alpha`: Defines the alpha parameter that set the importance of outer frontiers relative to the inner ones (see Eq. 5 in the [article](https://doi.org/10.1016/j.robot.2022.104179) for more details). We suggest to use *alpha = 0.5*.
- `potential_window_size`: Defines the size (in cells) of the local potential window used to navigate. We suggest to use *potential_window_size = 60*.
- `max_goal_radius`: Defines the desired distance (in cells) between the robot and the local goal. We suggest to use *max_goal_radius = 20*.
- `screen_x_init`: Defines the x axis start position for the visualization.
- `screen_y_init`: Defines the y axis start position for the visualization.
- `screen_scale_init`: Defines the start scale for the visualization.
- `screen_width_init`: Defines the width of window for the visualization.
- `screen_height_init`: Defines the height of window for the visualization.

## Keyboard commands

Some keyboard commands could be used to control the visualization interface and the robot.

### Control the visualization

- `esq`: Finish the exploration prematurely, without saving a map.
- `f`: Enable/Disable the exhibition of the local potential field.
- `c`: Locks and centers the camera in the robot (if the camera is already locked, it will disables the lock)
- `l`: Changes the LAEG visualization mode.
- `p`: Enable/Disable the exhibition of the path.
- `m`: Changes the map view (forward).
- `n`: Changes the map view (backward).
- `w`, `a`, `s`, `d`: Moves the cames in the four directions.
- `+`, `-`: Changes the zoom.

### Control the robot

- `1`: Mades the robot start to follow the potential provided by the exploration pipeline.
- `space`: Changes to manual control and stop the robot.
- `↑`, `↓`: Mades the robot go forward and backward in the manual control mode. 
- `←`, `→`: Mades the robot turn around its own center in the manual control mode.
  
