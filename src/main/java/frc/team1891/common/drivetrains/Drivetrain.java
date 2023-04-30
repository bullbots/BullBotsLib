// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.drivetrains;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1891.common.LazyDashboard;
import frc.team1891.common.Subsystem;

/** Drivetrain base. */
@SuppressWarnings("unused")
public abstract class Drivetrain extends Subsystem {
    protected final Field2d field = new Field2d();

    protected final DrivetrainConfig config;
    protected final Gyro gyro;

    protected boolean smartDashboardEnabled = false;

    /**
     * Creates a new Drivetrain
     * @param config the drivetrain config
     * @param gyro the gyro used by the drivetrian ({@link Gyro})
     */
    public Drivetrain(
        DrivetrainConfig config,
        Gyro gyro
    ) {
        this.config = config;
        this.gyro = gyro;

        resetGyro();

        SmartDashboard.putData("Drivetrain/Robot (Field2d)", field);
    }

    /**
     * Returns the speed of the robot as a {@link ChassisSpeeds}.
     * @return the speed of the robot
     */
    public abstract ChassisSpeeds getChassisSpeeds();

    public DrivetrainConfig getConfig() {
        return config;
    }

    public abstract Pose2d getPose2d();

    /**
     * Sets the gyro angle to 0.
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Sets the position of the odometry to 0, while maintaining the angle read from the gyro.
     */
    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    /**
     * Sets the position of the odometry to the given {@link Pose2d}, while maintaining the angle read from the gyro.
     * @param pose2d the pose the robot at after the reset
     */
    public abstract void resetOdometry(Pose2d pose2d);

    /**
     * Updates the odometry of the drivetrain.
     *
     * This is called by the periodic method. If the {@link Drivetrain#periodic()} method is overridden without calling
     * super.periodic() this will not work.
     */
    public abstract void updateOdometry();

    @Override
    protected void configureSmartDashboard() {
        LazyDashboard.addNumber("Drivetrain/gyroDegrees", gyro::getAngle);
        smartDashboardEnabled = true;
    }

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(getPose2d());
    }
}
