// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.team1891.common.drivetrains.DrivetrainConfig;

/**
 * Units are in meters and radians
 */
public class Constants {
    public static final double FIELD_WIDTH = Units.feetToMeters(27);
    public static final double FIELD_LENGTH = Units.feetToMeters(54);
    public static class DrivetrainConstants {
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
        public static final double DRIVETRAIN_DRIVE_GEAR_RATIO = 8.14; // MK4i L1
        // Dimensions
        public static final double WHEEL_BASE_LENGTH_METERS = Units.inchesToMeters(24 - (2.625 * 2));
        public static final double WHEEL_BASE_WIDTH_METERS = WHEEL_BASE_LENGTH_METERS; // It's a square

        public static final double rotationalP = 9;
        public static final double rotationalI = 0;
        public static final double rotationalD = 0;

        // HolonomicDriveController uses feedforward, which overshoots by a lot (at least in the sim), so only a D term is needed
        public static final double translationalP = 0;
        public static final double translationalI = 0;
        public static final double translationalD = 1;

        public static final double CHASSIS_MAX_VELOCITY = 3.;
        public static final double CHASSIS_MAX_ACCELERATION = 1.5;
        public static final double CHASSIS_MAX_ANGULAR_VELOCITY = Math.PI;
        public static final double CHASSIS_MAX_ANGULAR_ACCELERATION = 3 * Math.PI;
        public static final double MODULE_MAX_VELOCITY = 3.75; // Free speed max is ~3.96 for MK4i with L1

        public static final double driveP = 1.0;
        public static final double driveI = 0.0;
        public static final double driveD = 0.0;
        public static final double driveF = 0.0;

        public static final double steerP = 1.0;
        public static final double steerI = 0.0;
        public static final double steerD = 0.0;
        public static final double steerF = 0.0;

        public static final DrivetrainConfig CONFIG = new DrivetrainConfig(
            CHASSIS_MAX_VELOCITY,
            CHASSIS_MAX_ACCELERATION,
            CHASSIS_MAX_ANGULAR_VELOCITY,
            CHASSIS_MAX_ANGULAR_ACCELERATION,
            WHEEL_RADIUS_METERS,
            DRIVETRAIN_DRIVE_GEAR_RATIO,
            2048
        );

        public static class FrontLeft {
            public static final int DRIVE_CHANNEL = 1;
            public static final int STEER_CHANNEL = 2;
            public static final int CANCODER_CHANNEL = 9;
            public static final double ENCODER_OFFSET_DEGREES = 0;
        }
        public static class FrontRight {
            public static final int DRIVE_CHANNEL = 3;
            public static final int STEER_CHANNEL = 4;
            public static final int CANCODER_CHANNEL = 10;
            public static final double ENCODER_OFFSET_DEGREES = 0;
        }
        public static class BackLeft {
            public static final int DRIVE_CHANNEL = 5;
            public static final int STEER_CHANNEL = 6;
            public static final int CANCODER_CHANNEL = 11;
            public static final double ENCODER_OFFSET_DEGREES = 0;
        }
        public static class BackRight {
            public static final int DRIVE_CHANNEL = 7;
            public static final int STEER_CHANNEL = 8;
            public static final int CANCODER_CHANNEL = 12;
            public static final double ENCODER_OFFSET_DEGREES = 0;
        }
    }
}