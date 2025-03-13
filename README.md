# rosneuro_pong
Game of pong compatible with RosNeuro

## Overview
`rosneuro_pong` is a simple implementation of the classic Pong game, designed to be compatible with the RosNeuro framework. This package allows users to control the Pong game using neural signals processed by RosNeuro.

## Installation
To install the `rosneuro_pong` package, follow these steps:

2. Navigate to the workspace:
    ```bash
    cd /path/to/your/ros/workspace/src
    ```
1. Clone the repository:
    ```bash
    git clone https://github.com/yourusername/rosneuro_pong.git
    ```
3. Build the package:
    ```bash
    cd ..
    catkin_make
    ```
4. Source the workspace:
    ```bash
    source devel/setup.bash
    ```

## Usage
To include the node in a launch file is sufficient to add the line
``` xml
<node name='pong' pkg='rosneuro_pong' type='PongNode.py' output='screen'></node>
```

The node subscribes to the topics ```/smr/neuroprediction/player1``` ```/smr/neuroprediction/player1``` (rosneuro_msgs/NeuroFrame)

An example launch file is provided in the ```launch``` folder (requires [rosneuro_dl](https://github.com/neurorobotics-iaslab/rosneuro_dl)).

## Dependencies
This package depends on the following ROS packages:
- `rosneuro_msgs`

Make sure these packages are installed and properly configured in your ROS workspace.

## Contributing
Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgements
Special thanks to the RosNeuro team for their support and contributions to this project.