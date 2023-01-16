// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1891.common.drivetrains.Drivetrain;
import frc.team1891.common.drivetrains.DrivetrainConfig;
import frc.team1891.common.hardware.NavX;

/** Drivetrain base for any drivetrain with holonomic movement. */
public abstract class HolonomicDrivetrain extends Drivetrain {
    public HolonomicDrivetrain(ShuffleboardTab shuffleboardTab, DrivetrainConfig config, NavX gyro) {
        super(shuffleboardTab, config, gyro);
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

    public abstract void fromChassisSpeeds(ChassisSpeeds speeds);
}
