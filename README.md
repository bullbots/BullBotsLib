# BullBotsLib
## How to add to an FRC Project
### .jar
In your FRC project, and a folder called `libs`. A .jar file is available in 
[github releases](https://github.com/bullbots/BullBotsLib/releases).  Download it and add it to 
your lib folder.
### build.gradle
Inside your `build.gradle` file, add this to the dependencies:
```
dependencies {
    ...
    implementation fileTree(dir:'libs', include:'BullBotsLib-2023.2.2.jar')
}
```

## How to use in an FRC Project
### Drivetrains ([frc.team1891.common.drivetrains](https://github.com/bullbots/BullBotsLib/tree/2023.2.2/src/main/java/frc/team1891/common/drivetrains))
BullBotsLib provides 3 drivetrain types (but the ability to add your own by extending `Drivetrain` or 
`HolonomicDrivetrain`).  To use an existing drivetrain, create a class that extends it.

In your constructor you will need to call the super constructor and give it whatever parameters it needs.  It's easiest
to add all the parameters as private static fields in the top of your class (eg. the first parameter required:`private static final ShuffleboardTab
shuffuleboardTab = Shuffleboard.getTab("Drivetrain");`).

Once you do that, your drivetrain is ready to go.  All you need to do is configure your motors (eg. `talon.configFactoryDefault()`), as that's not done for you.

### User Input ([frc.team1891.common.control](https://github.com/bullbots/BullBotsLib/tree/2023.2.2/src/main/java/frc/team1891/common/control))
###### Triggers
`TriggerCombo`, `AxisTrigger`, and `POVTrigger` all behave similarly to a `JoystickButton`, in that you can attach commands that can be
scheduled when the trigger activates.  Hopefully they are simple and intuitive to use.
###### JoystickRotation2d
A simple class that converts two joystick axes into a `Rotation2d`.

The forward axis on a joystick is conventionally negative; you are responsible for inverting it before feeding it to
this class
###### SmartController
This is a wrapper class hoping to simplify the issues of controlling the robot with different kinds of controllers.  The
class detects what kind of controller you are using (using a small number of presets) and adjusts its functionality
slightly based on each.

One issue is the `SmartController` initializes upon robot startup.  Meaning, if you plug in your controller after the
robot turns on, it won't be detected.  A work around is to call `configure()` when the robot is enabled, or based on
some other trigger.

Though it's a little ugly, you can add custom button bindings to it as well by extending the class and overriding
configure().
```
if (getName().equals("Logitech Extreme 3D")) {
    setButton("resetGyro", 8);
} else if (getName().contains("Xbox Controller")) {
    setButton("resetGyro", XboxController.Button.kX.value);
} 
...
super.configure();
```
### Hardware ([frc.team1891.common.hardware](https://github.com/bullbots/BullBotsLib/tree/2023.2.2/src/main/java/frc/team1891/common/hardware))
###### NavX
The `NavX` is a simple wrapper class to `AHRS`, ensuring clarity on the units used by the gyro by implementing
`getDegrees()` and `getRadians()` instead of the built-in `getAngle()`.

`SimNavX` is extension of `NavX`, allowing for basic use in a simulator through setter methods, `setDegrees()` and
`setRadians()`.

###### WPI_CANSparkMax
This is a simple wrapper to `CANSparkMax`, making it a `Sendable`.  This is similar to `WPI_TalonFX`.
### Trajectories ([frc.team1891.common.trajectory](https://github.com/bullbots/BullBotsLib/tree/2023.2.2/src/main/java/frc/team1891/common/trajectory))
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