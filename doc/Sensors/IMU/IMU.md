# IMU
| Status | IMU Sensor |
| --- | --- |
| DRAFT | [NavXIMU](NavXIMU/NavXIMU.md) |

## Setup
```bash
sudo apt install ros-noetic-rviz-imu-plugin
```

## Visualization
```bash
rosrun rviz rviz -d src/fast_sensors/doc/Sensors/IMU/rviz_imu.rviz
```

## Software Design
![](../../output/Legend.png)

### Class Diagrams
![](../../../nodes/IMUNode/driver/doc/output/IMUNodeDriverClassDiagram.png)
![](../../../nodes/IMUNode/doc/output/IMUNodeClassDiagram.png)

### Sequence Diagrams
![](../../../nodes/IMUNode/driver/doc/output/IMUNodeDriverSequenceDiagram.png)
![](../../../nodes/IMUNode/doc/output/IMUNodeSequenceDiagram.png)

