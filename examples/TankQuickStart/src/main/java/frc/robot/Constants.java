// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.team1891.common.drivetrains.DrivetrainConfig;

public class Constants {
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
    public static final double TRACK_WIDTH_METERS = .2;
    public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 400/1.;

    public static final double CHASSIS_MAX_VELOCITY = 3.;
    public static final double CHASSIS_MAX_ACCELERATION = 1.5;
    public static final double CHASSIS_MAX_ANGULAR_VELOCITY = Math.PI;
    public static final double CHASSIS_MAX_ANGULAR_ACCELERATION = 3 * Math.PI;

    public static final DrivetrainConfig CONFIG = new DrivetrainConfig(
        CHASSIS_MAX_VELOCITY,
        CHASSIS_MAX_ACCELERATION,
        CHASSIS_MAX_ANGULAR_VELOCITY,
        CHASSIS_MAX_ANGULAR_ACCELERATION,
        WHEEL_RADIUS_METERS,
        DRIVETRAIN_DRIVE_GEAR_RATIO,
        2048
    );
    
    public static final int FRONT_LEFT_CAN_ID = 1;
    public static final int FRONT_RIGHT_CAN_ID = 2;
    public static final int BACK_LEFT_CAN_ID = 3;
    public static final int BACK_RIGHT_CAN_ID = 4;
}
