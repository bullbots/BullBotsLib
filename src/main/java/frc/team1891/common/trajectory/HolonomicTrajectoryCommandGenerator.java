// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.trajectory;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team1891.common.drivetrains.MecanumDrivetrain;
import frc.team1891.common.drivetrains.SwerveDrivetrain;

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
     * @param p
     * @param i
     * @param d
     */
    public static void setRotationalPID(double p, double i, double d) {
        rP = p;
        rI = i;
        rD = d;
    }

    /**
     * PID values to control the translational movement of the robot along the swerve trajectory.
     * @param p
     * @param i
     * @param d
     */
    public static void setTranslationalPID(double p, double i, double d) {
        tP = p;
        tI = i;
        tD = d;
    }
    
    /**
     * Creates an autonomous {@link SequentialCommandGroup} that follows a {@link Trajectory} for swerve drive, then stops.
     * @param drivetrain subsystem
     * @param initialHeading intitial angle of the robot
     * @param finalHeading final angle of the robot
     * @param pointTranslations points for the trajectory to go through
     * @return an autonomous command
     */
    public static SequentialCommandGroup generate(SwerveDrivetrain drivetrain, Rotation2d initialHeading, Rotation2d finalHeading, Translation2d ... pointTranslations) {
        ArrayList<Translation2d> points = new ArrayList<>();
        Collections.addAll(points, pointTranslations);

        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
        ).setKinematics(drivetrain.getKinematics());

        Translation2d first = points.remove(0);
        Translation2d last = points.remove(points.size()-1);
        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            new Pose2d(first, initialHeading),
            points,
            new Pose2d(last, finalHeading),
            config
        );

        ProfiledPIDController headingController = new ProfiledPIDController(rP, rI, rD,
            new TrapezoidProfile.Constraints(
                drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
                drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
            )
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicTrajectoryCommand command = new HolonomicTrajectoryCommand(
            trajectory,
            drivetrain::getPose2d,
            new PIDController(tP, tI, tD),
            new PIDController(tP, tI, tD),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        // drivetrain.resetOdometry(trajectory.getInitialPose());

        return command
            .beforeStarting(() -> {
                drivetrain.resetGyro();
                drivetrain.resetOdometry(trajectory.getInitialPose());
            }, drivetrain)
            .andThen(() -> {drivetrain.stop();});
    }

    public static SequentialCommandGroup generate(SwerveDrivetrain drivetrain, Pose2d ... poses) {
        // List<Translation2d> points = Arrays.asList(pointTranslations);
        ArrayList<Pose2d> points = new ArrayList<>();
        Collections.addAll(points, poses);

        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            points,
            config
        );

        ProfiledPIDController headingController = new ProfiledPIDController(rP, rI, rD,
            new TrapezoidProfile.Constraints(
                drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
                drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
            )
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicTrajectoryCommand command = new HolonomicTrajectoryCommand(
            trajectory,
            drivetrain::getPose2d,
            new PIDController(tP, tI, tD),
            new PIDController(tP, tI, tD),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        return command
            .beforeStarting(() -> {
                drivetrain.resetGyro();
                drivetrain.resetOdometry(trajectory.getInitialPose());
            }, drivetrain)
            .andThen(() -> {drivetrain.stop();});
    }

    @SafeVarargs
    public static SequentialCommandGroup generate(SwerveDrivetrain drivetrain, Pair<Pose2d, Rotation2d> ... posesAndHeadings) {
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
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            points,
            headings,
            config
        );

        ProfiledPIDController headingController = new ProfiledPIDController(rP, rI, rD,
            new TrapezoidProfile.Constraints(
                drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
                drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
            )
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicTrajectoryCommand command = new HolonomicTrajectoryCommand(
            trajectory,
            drivetrain::getPose2d,
            new PIDController(tP, tI, tD),
            new PIDController(tP, tI, tD),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        return command
            .beforeStarting(() -> {
                drivetrain.resetGyro();
                drivetrain.resetOdometry(trajectory.getInitialPose());
            }, drivetrain)
            .andThen(() -> {drivetrain.stop();});
    }

        /**
     * Creates an autonomous {@link SequentialCommandGroup} that follows a {@link Trajectory} for swerve drive, then stops.
     * @param drivetrain subsystem
     * @param initialHeading intitial angle of the robot
     * @param finalHeading final angle of the robot
     * @param pointTranslations points for the trajectory to go through
     * @return an autonomous command
     */
    public static SequentialCommandGroup generate(MecanumDrivetrain drivetrain, Rotation2d initialHeading, Rotation2d finalHeading, Translation2d ... pointTranslations) {
        ArrayList<Translation2d> points = new ArrayList<>();
        Collections.addAll(points, pointTranslations);

        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
        ).setKinematics(drivetrain.getKinematics());

        Translation2d first = points.remove(0);
        Translation2d last = points.remove(points.size()-1);
        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            new Pose2d(first, initialHeading),
            points,
            new Pose2d(last, finalHeading),
            config
        );

        ProfiledPIDController headingController = new ProfiledPIDController(rP, rI, rD,
            new TrapezoidProfile.Constraints(
                drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
                drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
            )
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicTrajectoryCommand command = new HolonomicTrajectoryCommand(
            trajectory,
            drivetrain::getPose2d,
            new PIDController(tP, tI, tD),
            new PIDController(tP, tI, tD),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        // drivetrain.resetOdometry(trajectory.getInitialPose());

        return command
            .beforeStarting(() -> {
                drivetrain.resetGyro();
                drivetrain.resetOdometry(trajectory.getInitialPose());
            }, drivetrain)
            .andThen(() -> {drivetrain.stop();});
    }

    public static SequentialCommandGroup generate(MecanumDrivetrain drivetrain, Pose2d ... poses) {
        // List<Translation2d> points = Arrays.asList(pointTranslations);
        ArrayList<Pose2d> points = new ArrayList<>();
        Collections.addAll(points, poses);

        TrajectoryConfig config = new TrajectoryConfig(
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            points,
            config
        );

        ProfiledPIDController headingController = new ProfiledPIDController(rP, rI, rD,
            new TrapezoidProfile.Constraints(
                drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
                drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
            )
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicTrajectoryCommand command = new HolonomicTrajectoryCommand(
            trajectory,
            drivetrain::getPose2d,
            new PIDController(tP, tI, tD),
            new PIDController(tP, tI, tD),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        return command
            .beforeStarting(() -> {
                drivetrain.resetGyro();
                drivetrain.resetOdometry(trajectory.getInitialPose());
            }, drivetrain)
            .andThen(() -> {drivetrain.stop();});
    }

    @SafeVarargs
    public static SequentialCommandGroup generate(MecanumDrivetrain drivetrain, Pair<Pose2d, Rotation2d> ... posesAndHeadings) {
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
            drivetrain.getConfig().chassisMaxVelocityMetersPerSecond,
            drivetrain.getConfig().chassisMaxAccelerationMetersPerSecondSquared
        ).setKinematics(drivetrain.getKinematics());

        HolonomicTrajectory trajectory = HolonomicTrajectoryGenerator.generateHolonomicTrajectory(
            points,
            headings,
            config
        );

        ProfiledPIDController headingController = new ProfiledPIDController(rP, rI, rD,
            new TrapezoidProfile.Constraints(
                drivetrain.getConfig().chassisMaxAngularVelocityRadiansPerSecond,
                drivetrain.getConfig().chassisMaxAngularAccelerationRadiansPerSecondSquared
            )
        );
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicTrajectoryCommand command = new HolonomicTrajectoryCommand(
            trajectory,
            drivetrain::getPose2d,
            new PIDController(tP, tI, tD),
            new PIDController(tP, tI, tD),
            headingController,
            drivetrain::fromChassisSpeeds,
            drivetrain
        );

        return command
            .beforeStarting(() -> {
                drivetrain.resetGyro();
                drivetrain.resetOdometry(trajectory.getInitialPose());
            }, drivetrain)
            .andThen(() -> {drivetrain.stop();});
    }
}
