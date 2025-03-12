# NavX IMU
## Usage Instructions
### IMU Node

#### Configuration


### Test Driver
A Test Executable for the Sonar Array Node Driver can be ran by running:
```bash
./install/bin/exec_navvximu_driver
```

This will give output similiar to:
```bash
Tester for NavX IMU Driver
-h This Menu.
-d Device.  Default: /dev/ttyACM0
-l Logger Threshold. [DEBUG,INFO,NOTICE,WARN,ERROR]

```

Examples:
```bash
./install/bin/exec_navvximu_driver -d /dev/ttyACM0 -l DEBUG

```

## References
- [Datasheet](ref/navX_MXP_Datasheet.pdf)
- [User Guide](ref/navX-MXP%20Robotics%20Navigation%20Sensor%20User%20Guide.pdf)