// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.team1891.common.LazyDashboard;

/** Drivetrain base for any drivetrain with holonomic movement. */
@SuppressWarnings("unused")
public abstract class HolonomicDrivetrain extends Drivetrain {
    public HolonomicDrivetrain(DrivetrainConfig config, Gyro gyro) {
        super(config, gyro);
    }

    /**
     * Primary method of controlling the robot.
     * 
     * @param xSpeed forward speed
     * @param ySpeed leftward speed
     * @param rot rotational speed
     * @param fieldRelative field or robot relative
     */
    public abstract void holonomicDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative);

    /**
     * Drives the robot to the desired field oriented {@link ChassisSpeeds}.
     * @param fieldSpeeds field oriented chassis speeds
     */
    public void fromFieldSpeeds(ChassisSpeeds fieldSpeeds) {
        fromChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, gyro.getRotation2d()));
    }

    @Override
    protected void configureSmartDashboard() {
        super.configureSmartDashboard();
        LazyDashboard.addNumber("Drivetrain/xSpeed (Meters per Second)", () -> getChassisSpeeds().vxMetersPerSecond);
        LazyDashboard.addNumber("Drivetrain/ySpeed (Meters per Second)", () -> getChassisSpeeds().vyMetersPerSecond);
        LazyDashboard.addNumber("Drivetrain/omegaSpeed (Radians per Second)", () -> getChassisSpeeds().omegaRadiansPerSecond);
    }
}
