// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team1891.common.trajectory.HolonomicTrajectory.State;

import java.util.function.Consumer;
import java.util.function.Supplier;

/** 
 * A command that follows a {@link HolonomicTrajectory}.
 */
public class HolonomicTrajectoryCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final HolonomicTrajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final RotatingHolonomicDriveController controller;
    private final Consumer<ChassisSpeeds> outputChassisSpeeds;
    // private final Supplier<Rotation2d> m_desiredRotation;

    private final Field2d field2d = new Field2d();

    /**
     * Constructs a new SwerveControllerCommand that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
     * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param pose A function that supplies the robot pose - use one of the odometry classes to
     *     provide this.
     * @param xController The Trajectory Tracker PID controller for the robot's x position.
     * @param yController The Trajectory Tracker PID controller for the robot's y position.
     * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
     * @param requirements The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public HolonomicTrajectoryCommand(
            HolonomicTrajectory trajectory,
            Supplier<Pose2d> pose,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Subsystem... requirements) {
        this.trajectory = trajectory;
        this.pose = pose;

        controller =
            new RotatingHolonomicDriveController(
                xController,
                yController,
                thetaController
            );

        this.outputChassisSpeeds =
            outputChassisSpeeds;

        addRequirements(requirements);

        SmartDashboard.putData("Holonomic Trajectory (Field2d)", field2d);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        field2d.getObject("Holonomic Trajectory Path").setTrajectory(trajectory.getAsTrajectory());

        controller.reset();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = timer.get();
        State desiredState = trajectory.sample(curTime);

        ChassisSpeeds targetChassisSpeeds =
            controller.calculate(pose.get(), desiredState);
        
        outputChassisSpeeds.accept(targetChassisSpeeds);

        field2d.setRobotPose(pose.get());
        field2d.getObject("Desired State").setPose(desiredState.poseMeters);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        outputChassisSpeeds.accept(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
  }
