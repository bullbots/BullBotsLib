# BullBotsLib
## How to add to an FRC Project
### .jar
In your FRC project, add a folder called `libs`. A .jar file is available in 
[github releases](https://github.com/bullbots/BullBotsLib/releases).  Download it and add it to 
your lib folder.
### build.gradle
Inside your `build.gradle` file, add this to the dependencies:
```
dependencies {
    ...
    implementation fileTree(dir:'libs', include:'BullBotsLib-**version**.jar')
}
```

## How to use in an FRC Project
### Drivetrains ([frc.team1891.common.drivetrains](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/drivetrains))
BullBotsLib provides 3 drivetrain types (but the ability to add your own by extending `Drivetrain` or 
`HolonomicDrivetrain`)  in order to remove code that would be repeated year to year, and make things as easy as 
possible when getting started.  To use an existing drivetrain, create a class that extends it.

In your constructor you will need to call the super constructor and give it whatever parameters it needs.  It's easiest
to add all the parameters as private static fields in the top of your class.

Once you do that, your drivetrain is ready to go.  All you need to do is configure your motors 
(e.g. `talon.configFactoryDefault()`), as that's not done for you.

You can also call `configureSmartDashboard()` in the constructor (as you can with anything that extends `Subsystem` from
BullBotsLib) to get extra diagnostic information

###### Swerve
`SwerveDrivetrain` tries to make it as easy as possible to get started with Swerve.  Each `SwerveModule` holds a 
`DriveController` and a `SteerController`, which separates the control over a drive motor and steer motor for easy 
changes.  These classes and basic implementations are found under [frc.team1891.common.drivetrains.swervecontrollers](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/drivetrains/swervecontrollers).

### User Input ([frc.team1891.common.control](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/control))
###### Triggers
`AxisTrigger` and `POVTrigger` behave similarly to a `JoystickButton`, in that you can attach commands that can be
scheduled when the trigger activates, but activate based one something other than a simple button press.
###### JoystickRotation2d
A simple class that converts two joystick axes into a `Rotation2d`.

The forward axis on a joystick is conventionally negative; you are responsible for inverting it before feeding it to
this class
###### HID Devices
`Guitar` is a simple user input device that can be connected to a Guitar Hero controller.

`X52ProfessionalHOTAS` is a slightly less simple device, giving the driver access to more buttons than they could ever
need.

### Hardware ([frc.team1891.common.hardware](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/hardware))
###### NavX
The `NavX` is a simple wrapper class to `AHRS`, ensuring clarity on the units used by the gyro by implementing
`getDegrees()` and `getRadians()` instead of the built-in `getAngle()`.

`SimNavX` is extension of `NavX`, allowing for basic use in a simulator through setter methods, `setDegrees()` and
`setRadians()`.

###### WPI_CANSparkMax
This is a simple wrapper to `CANSparkMax`, making it a `Sendable`.  This is similar to `WPI_TalonFX`.

###### Lazy Motor Controllers and Solenoids
Under [frc.team1891.common.hardware.lazy](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/hardware/lazy)
there are wrapper classes that override the "set" methods of the given motor controllers and solenoids (e.g. TalonFX or
Solenoid).  The new methods help reduce the weight on the CAN bus from repeated calls of the same command by only
calling the super method when the method parameters are different from before.

### LEDs ([frc.team1891.common.led](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/led))
The `LEDString` and `LEDMatrix` both exist in order to make controlling LEDs as clean as possible, even with complex 
animations.  

The `LEDPattern` interfaces within each class help with that.  Within each class there is also an `LEDPatterns` class,
which just holds a few example patterns that you can easily use.

See the [LEDMatrix example](https://github.com/bullbots/BullBotsLib/tree/main/examples/LEDMatrixWithModes) for more 
information on how to use it.  The only difference between `LEDMatrix` and `LEDString` is that the matrix is meant for
an LED grid, and supports using x and y coordinates to address individual LEDs.

The `LEDStringSegment` makes it easy to only apply an `LEDPattern` to a certain segment of an LED strip. See the 
[Segmented LEDString example](https://github.com/bullbots/BullBotsLib/tree/main/examples/SegmentedLEDString)

### Logging ([frc.team1891.common.logger](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/logger))
The `BullLogger` is a logger class that uses the `DataLog` class.  It exports its logs to a USB drive plugged into the
roboRIO and optionally also outputs to the console.

### Trajectories ([frc.team1891.common.trajectory](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/trajectory))
The purpose of this package is to make an easy way to generate a trajectory (`HolonomicTrajectory`) that allows a robot to move in one direction
while facing another direction.

To generate a trajectory, simply call `HolonomicTrajectoryGenerator.generateHolonomicTrajectory(...)`.  There are several
method signatures depending on exactly what you need to do, but
`generateHolonomicTrajectory(List<Pose2d> waypoints, List<Rotation2d> headings, TrajectoryConfig config)` is
recommended.

To generate a command that follows a trajectory like the one above, use
`HolonomicTrajectoryCommandGenerator.generate(...)`.  This time recommending using
`generate(SwerveDrivetrain drivetrain, Pair<Pose2d, Rotation2d> ... posesAndHeadings)` as it allows for the intended
moving in one direction while facing another, with the highest level of control.  For each waypoint you want the robot
to pass through, add a `Pair`, with a `Pose2d` representing the position and direction of movement, and a `Rotation2d`
representing the heading of the robot while it moves through that point.

If you are generating commands as described above, you will need to tune PID values to your specific robot.  This can be
done by calling `HolonomicTrajectoryCommandGenerator.setTranslationalPID(double p, double i, double d)` and
`HolonomicTrajectoryCommandGenerator.setRotationalPID(double p, double i, double d)`.

### Vision ([frc.team1891.common.vision](https://github.com/bullbots/BullBotsLib/tree/main/src/main/java/frc/team1891/common/vision))
###### Limelight
The `Limelight` class wraps the NetworkTables used by Limelight into a more intuitive structure, adding features such as
`isConnected()`, `getLatencyMs()`, and `setPipeline(int pipeline)`
