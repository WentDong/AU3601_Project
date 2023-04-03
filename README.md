# AU3601_Project
Project of Course AU3601, 2023 Spring, SJTU

It contains all codes of the whole course.



## For Final Project

### Codes Implemented

-   `catkin_ws/src/cylinder_robot/scripts/controller.py`
-   `catkin_ws/src/cylinder_robot/scripts/driver.py`
-   `catkin_ws/src/cylinder_robot/scripts/perception.py`

### Setting Modified

-   `catkin_ws/src/cylinder_robot/worlds/cylinder.world`

### Build

```bash
cd catkin_ws
catkin_make
source ./devel/setup.bash
```

### Run

```
roslaunch cylinder_robot runCylinder.launch 
```



