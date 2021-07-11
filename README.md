# interactive_stl_marker

## implementation

- First, please clone this repository.

```bash
cd /catkin_ws/src
git clone git@github.com:SOutaHI/interactive_stl_marker.git
```

- Second, build this package

```bash
cd ../
catkin build 
source devel/setup.bash
```

- launch

```bash
roslaunch interactive_stl_marker generate_interactive_markers.launch
```

<p align="center">
    <img src="./resources/test.png?"width="750" height="436">
</p>

- If you change stl file path or initialize position and scale, Please check config/param.yaml.