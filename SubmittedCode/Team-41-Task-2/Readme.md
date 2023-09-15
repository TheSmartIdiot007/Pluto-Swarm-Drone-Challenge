# Task 2
### Make the Pluto1.2 drone follow a trajectory of a rectangle with visual feedback 
## Our Solution
* Used python wrapper **API** to communicate with the drone and control it
* Used **OpenCV** to detect the **Aruco** marker on the drone to get its position for feedback
* Generated Waypoints for the drone to follow using **Rectangle** class
* Used **PID** controller to control the drone to follow the waypoints

### Code Structure and Documentation
1. control folder contains the code for the PID controller, task2, Rectangle class which is used to control the drone to follow the waypoints, and slider class for real time **GUI** based PID tuning of Pluto [Documentation](control/documentation.md)
2. DISTANCE_ESTIMATION folder of CV contains the code for the Aruco marker detection and pose estimation. [Documentation](CV/DISTANCE_ESTIMATION/documentation.md)
3. CAMERA-CALIBRATION folder of CV contains the code for the camera calibration. [Documentation](CV/CAMERA-CALIBRATION/documentation.md)
4. python_wrapper folder contains the code for the python wrapper API. [Documentation](python_wrapper/documentation.md), [Example](python_wrapper/src/example.md)

## Instructions to run Task 2
Go to the `control/src` folder and run the following command in terminal after installing all the required libraries in the `requirements.txt` file

```bash
python control_node.py
```

* To make the drone hover make the boolean variable `hover` in `if __name__ == '__main__' :` of the `control_node.py` file to `True`
* To follow the waypoints make the boolean variable `hover` in `if __name__ == '__main__' :` of the `control_node.py` file to `False`