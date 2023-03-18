// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.trajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1891.common.drivetrains.MecanumDrivetrain;
import frc.team1891.common.drivetrains.SwerveDrivetrain;

import java.util.ArrayList;
import java.util.Collections;

@SuppressWarnings("unused")
public class HolonomicTrajectoryCommandGenerator {
    private HolonomicTrajectoryCommandGenerator() {}
    
    // Rotational PID
    private static double rP = 1,
                          rI = 0,
                          rD = 0;
    // Translational PID
    private static double tP = 1,
                          tI = 0,
                          tD = 0;

    /**
     * PID values to control the rotation of the robot along the swerve trajectory.
     * @param p coefficient
     * @param i coefficient
     * @param d coefficient
     */
    public static void setRotationalPID(double p, double i, double d) {
        rP = p;
        rI = i;
        rD = d;
    }

    /**
     * PID values to control the translational movement of the robot along the swerve trajectory.
     * @param p coefficient
     * @param i coefficient
     * @param d coefficient
     */
    public static void setTranslationalPID(double p, double i, double d) {
        tP = p;
        tI = i;
        tD = d;
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param trajectory the trajectory to follow
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(SwerveDrivetrain drivetrain, boolean resetPoseBeforeStarting, HolonomicTrajectory trajectory) {
        return generate(drivetrain, resetPoseBeforeStarting, true, trajectory);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param stopOnFinish should the drivetrain be told to stop when it finishes
     * @param trajectory the trajectory to follow
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(SwerveDrivetrain drivetrain, boolean resetPoseBeforeStarting, boolean stopOnFinish, HolonomicTrajectory trajectory) {
        ProfiledPIDController headingController = new ProfiledPIDController(rP, rI, rD,
                new TrapezoidProfile.Constraints(
                        drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond(),
                        drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared()
                )
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        Command command = new HolonomicTrajectoryCommand(
                trajectory,
                drivetrain::getPose2d,
                new PIDController(tP, tI, tD),
                new PIDController(tP, tI, tD),
                headingController,
                drivetrain::fromChassisSpeeds,
                drivetrain
        );

        if (resetPoseBeforeStarting) {
            command = command.beforeStarting(() -> drivetrain.resetOdometry(trajectory.getInitialPose()));
        }
        if (stopOnFinish) {
            command = command.andThen(drivetrain::stop);
        }

        return command;
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param initialHeading initial angle of the robot
     * @param finalHeading final angle of the robot
     * @param pointTranslations points for the trajectory to go through
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(SwerveDrivetrain drivetrain, boolean resetPoseBeforeStarting, Rotation2d initialHeading, Rotation2d finalHeading, Translation2d ... pointTranslations) {
        return generate(drivetrain, resetPoseBeforeStarting, true, initialHeading, finalHeading, pointTranslations);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param stopOnFinish should the drivetrain be told to stop when it finishes
     * @param initialHeading initial angle of the robot
     * @param finalHeading final angle of the robot
     * @param pointTranslations points for the trajectory to go through
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(SwerveDrivetrain drivetrain, boolean resetPoseBeforeStarting, boolean stopOnFinish, Rotation2d initialHeading, Rotation2d finalHeading, Translation2d[] pointTranslations) {
        ArrayList<Translation2d> points = new ArrayList<>();
        Collections.addAll(points, pointTranslations);

        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond(),
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared()
        ).setKinematics(drivetrain.getKinematics());

        Translation2d first = points.remove(0);
        Translation2d last = points.remove(points.size()-1);
        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            new Pose2d(first, initialHeading),
            points,
            new Pose2d(last, finalHeading),
            config
        );

        return generate(drivetrain, resetPoseBeforeStarting, stopOnFinish, trajectory);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it's already at the initial pose
     * @param poses the waypoints for the robot to drive through; the rotation of the pose controls the chassis heading and direction of movement
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(SwerveDrivetrain drivetrain, boolean resetPoseBeforeStarting, Pose2d ... poses) {
        return generate(drivetrain, resetPoseBeforeStarting, true, poses);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it's already at the initial pose
     * @param stopOnFinish should the drivetrain be told to stop when it finishes
     * @param poses the waypoints for the robot to drive through; the rotation of the pose controls the chassis heading and direction of movement
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(SwerveDrivetrain drivetrain, boolean resetPoseBeforeStarting, boolean stopOnFinish, Pose2d ... poses) {
        ArrayList<Pose2d> points = new ArrayList<>();
        Collections.addAll(points, poses);

        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond(),
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared()
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            points,
            config
        );

        return generate(drivetrain, resetPoseBeforeStarting, stopOnFinish, trajectory);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param posesAndHeadings pairs containing a pose that controls the waypoints and direction of movement through said waypoints, and a rotation that controls the heading of the chassis at each point
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    @SafeVarargs
    public static Command generate(SwerveDrivetrain drivetrain, boolean resetPoseBeforeStarting, Pair<Pose2d, Rotation2d> ... posesAndHeadings) {
        return generate(drivetrain, resetPoseBeforeStarting, true, posesAndHeadings);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param stopOnFinish should the drivetrain be told to stop when it finishes
     * @param posesAndHeadings pairs containing a pose that controls the waypoints and direction of movement through said waypoints, and a rotation that controls the heading of the chassis at each point
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    @SafeVarargs
    public static Command generate(SwerveDrivetrain drivetrain, boolean resetPoseBeforeStarting, boolean stopOnFinish, Pair<Pose2d, Rotation2d> ... posesAndHeadings) {
        Pose2d[] poses = new Pose2d[posesAndHeadings.length];
        Rotation2d[] rotations = new Rotation2d[posesAndHeadings.length];
        for (int i = 0; i < posesAndHeadings.length; i++) {
            poses[i] = posesAndHeadings[i].getFirst();
            rotations[i] = posesAndHeadings[i].getSecond();
        }
        // List<Translation2d> points = Arrays.asList(pointTranslations);
        ArrayList<Pose2d> points = new ArrayList<>();
        Collections.addAll(points, poses);
        ArrayList<Rotation2d> headings = new ArrayList<>();
        Collections.addAll(headings, rotations);

        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond(),
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared()
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            points,
            headings,
            config
        );

        return generate(drivetrain, resetPoseBeforeStarting, stopOnFinish, trajectory);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for swerve drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param trajectory the trajectory to follow
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(MecanumDrivetrain drivetrain, boolean resetPoseBeforeStarting, HolonomicTrajectory trajectory) {
        return generate(drivetrain, resetPoseBeforeStarting, true, trajectory);
    }

    public static Command generate(MecanumDrivetrain drivetrain, boolean resetPoseBeforeStarting, boolean stopOnFinish, HolonomicTrajectory trajectory) {
        ProfiledPIDController headingController = new ProfiledPIDController(rP, rI, rD,
                new TrapezoidProfile.Constraints(
                        drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond(),
                        drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared()
                )
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        Command command = new HolonomicTrajectoryCommand(
                trajectory,
                drivetrain::getPose2d,
                new PIDController(tP, tI, tD),
                new PIDController(tP, tI, tD),
                headingController,
                drivetrain::fromChassisSpeeds,
                drivetrain
        );

        if (resetPoseBeforeStarting) {
            command = command.beforeStarting(() -> drivetrain.resetOdometry(trajectory.getInitialPose()));
        }
        if (stopOnFinish) {
            command = command.andThen(drivetrain::stop);
        }

        return command;
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for mecanum drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param initialHeading initial angle of the robot
     * @param finalHeading final angle of the robot
     * @param pointTranslations points for the trajectory to go through
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(MecanumDrivetrain drivetrain, boolean resetPoseBeforeStarting, Rotation2d initialHeading, Rotation2d finalHeading, Translation2d ... pointTranslations) {
        return generate(drivetrain, resetPoseBeforeStarting, true, initialHeading, finalHeading, pointTranslations);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} for mecanum drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param stopOnFinish should the drivetrain be told to stop when it finishes
     * @param initialHeading initial angle of the robot
     * @param finalHeading final angle of the robot
     * @param pointTranslations points for the trajectory to go through
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(MecanumDrivetrain drivetrain, boolean resetPoseBeforeStarting, boolean stopOnFinish, Rotation2d initialHeading, Rotation2d finalHeading, Translation2d[] pointTranslations) {
        ArrayList<Translation2d> points = new ArrayList<>();
        Collections.addAll(points, pointTranslations);

        TrajectoryConfig config = new TrajectoryConfig(
                drivetrain.getConfig().chassisMaxVelocityMetersPerSecond(),
                drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared()
        ).setKinematics(drivetrain.getKinematics());

        Translation2d first = points.remove(0);
        Translation2d last = points.remove(points.size()-1);
        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
                new Pose2d(first, initialHeading),
                points,
                new Pose2d(last, finalHeading),
                config
        );

        return generate(drivetrain, resetPoseBeforeStarting, stopOnFinish, trajectory);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for mecanum drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it's already at the initial pose
     * @param poses the waypoints for the robot to drive through; the rotation of the pose controls the chassis heading and direction of movement
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(MecanumDrivetrain drivetrain, boolean resetPoseBeforeStarting, Pose2d ... poses) {
        return generate(drivetrain, resetPoseBeforeStarting, true, poses);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} for mecanum drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it's already at the initial pose
     * @param stopOnFinish should the drivetrain be told to stop when it finishes
     * @param poses the waypoints for the robot to drive through; the rotation of the pose controls the chassis heading and direction of movement
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    public static Command generate(MecanumDrivetrain drivetrain, boolean resetPoseBeforeStarting, boolean stopOnFinish, Pose2d ... poses) {
        // List<Translation2d> points = Arrays.asList(pointTranslations);
        ArrayList<Pose2d> points = new ArrayList<>();
        Collections.addAll(points, poses);

        TrajectoryConfig config = new TrajectoryConfig(
                drivetrain.getConfig().chassisMaxVelocityMetersPerSecond(),
                drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared()
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
                points,
                config
        );

        return generate(drivetrain, resetPoseBeforeStarting, stopOnFinish, trajectory);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} and stops for mecanum drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param posesAndHeadings pairs containing a pose that controls the waypoints and direction of movement through said waypoints, and a rotation that controls the heading of the chassis at each point
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    @SafeVarargs
    public static Command generate(MecanumDrivetrain drivetrain, boolean resetPoseBeforeStarting, Pair<Pose2d, Rotation2d> ... posesAndHeadings) {
        return generate(drivetrain, resetPoseBeforeStarting, true, posesAndHeadings);
    }

    /**
     * Creates a {@link HolonomicTrajectoryCommand} that follows a {@link HolonomicTrajectory} for mecanum drive.
     * @param drivetrain subsystem
     * @param resetPoseBeforeStarting when true the robot assumes it is already at the initial pose
     * @param stopOnFinish should the drivetrain be told to stop when it finishes
     * @param posesAndHeadings pairs containing a pose that controls the waypoints and direction of movement through said waypoints, and a rotation that controls the heading of the chassis at each point
     * @return a command that follows a {@link HolonomicTrajectory}
     */
    @SafeVarargs
    public static Command generate(MecanumDrivetrain drivetrain, boolean resetPoseBeforeStarting, boolean stopOnFinish, Pair<Pose2d, Rotation2d> ... posesAndHeadings) {
        Pose2d[] poses = new Pose2d[posesAndHeadings.length];
        Rotation2d[] rotations = new Rotation2d[posesAndHeadings.length];
        for (int i = 0; i < posesAndHeadings.length; i++) {
            poses[i] = posesAndHeadings[i].getFirst();
            rotations[i] = posesAndHeadings[i].getSecond();
        }
        // List<Translation2d> points = Arrays.asList(pointTranslations);
        ArrayList<Pose2d> points = new ArrayList<>();
        Collections.addAll(points, poses);
        ArrayList<Rotation2d> headings = new ArrayList<>();
        Collections.addAll(headings, rotations);

        TrajectoryConfig config = new TrajectoryConfig(
                drivetrain.getConfig().chassisMaxVelocityMetersPerSecond(),
                drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared()
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
                points,
                headings,
                config
        );

        return generate(drivetrain, resetPoseBeforeStarting, stopOnFinish, trajectory);
    }
}
