# Cognex Push Back Code
## Project Structure
All the main robot-related files are located in the `robot` folder. This folder is where autos and subsystem code gets written.

# Configuration
To configure ports go to `robot/globals.cpp`, where most devices are already listed. If you need to add a device go to `robot/globals.h` and add the device there as `cpp extern pros::Device my_device;` then define it in `robot/globals.cpp` as you would with other devices.

### Note
Appart from setting the ports you should also set the other settings in `robot/globals.cpp`!

Especially important are `drivetrain_config` and the distance sensor offsets, as they are strictly robot specific and should be set every time the robot changes. The odometry offsets should also be changed


## Figuring out offsets
### Odometry
[Lemlib's guide](https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html) can be used to figure out the offsets. However, the offsets have the OPPOSITE sign compared to lemlib's. Otherwise offsets are the same.
### Distance Sensor Offsets
These offsets are measured from the center of the robot. The x-coordinate represents the front-back axis of the robot, where the back of the robot is negative and the front of the robot is positive. Similarly, the y-coordinate represents the left-right axis of the robot, where the left is negative and the right is positive.

# Writing subsystems
The intake subsystem that is currently present is a good place to start from, and all subsystems should have a similar structure.

# Writing Autos
The auto1 auto is a pretty basic example of a working. To make another auto just copy auto1 into another file and rename the `namespace auton1` into whatever your auton will be. Then, go to `robot/include/autos.h` and define a `NEW_AUTON(whatever the new auton is)`. To get it to show up on the selector go to `robot/src/autonomous.cpp` and add the auton with `AUTON(whatever the new auton is, "cool auton name")`.
