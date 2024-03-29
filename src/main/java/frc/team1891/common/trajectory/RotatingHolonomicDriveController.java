// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.trajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Recreation of {@link HolonomicDriveController} to allow for free rotation of the chassis. */
@SuppressWarnings("unused")
public class RotatingHolonomicDriveController {
    private static boolean smartDashboardEnabled = false;
    /**
     * SmartDashboard values for debug.
     * @param enable show or hide SmartDashboard values
     */
    public static void enableSmartDashboard(boolean enable) {
        smartDashboardEnabled = enable;
    }

    private Pose2d poseError = new Pose2d();
    private Rotation2d rotationError = new Rotation2d();
    private Pose2d poseTolerance = new Pose2d();
    private boolean enabled = true;
  
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
  
    private boolean firstRun = true;
  
    /**
     * Constructs a holonomic drive controller.
     *
     * @param xController A PID Controller to respond to error in the field-relative x direction.
     * @param yController A PID Controller to respond to error in the field-relative y direction.
     * @param thetaController A profiled PID controller to respond to error in angle.
     */
    @SuppressWarnings("ParameterName")
    public RotatingHolonomicDriveController(
            PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
    }
  
    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        final var eTranslate = poseError.getTranslation();
        final var eRotate = rotationError;
        final var tolTranslate = poseTolerance.getTranslation();
        final var tolRotate = poseTolerance.getRotation();
        return Math.abs(eTranslate.getX()) < tolTranslate.getX()
            && Math.abs(eTranslate.getY()) < tolTranslate.getY()
            && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }
  
    /**
     * Sets the pose error which is considered tolerance for use with atReference().
     *
     * @param tolerance The pose error which is tolerable.
     */
    public void setTolerance(Pose2d tolerance) {
        poseTolerance = tolerance;
    }
  
    /**
     * Returns the next output of the holonomic drive controller.
     *
     * @param currentPose The current pose.
     * @param poseRef The desired pose.
     * @param linearVelocityRefMeters The linear velocity reference.
     * @param angleRef The angular reference.
     * @return The next output of the holonomic drive controller.
     */
    @SuppressWarnings("LocalVariableName")
    public ChassisSpeeds calculate(
            Pose2d currentPose, Pose2d poseRef, Rotation2d directionOfMovement, double linearVelocityRefMeters, Rotation2d angleRef) {
        // If this is the first run, then we need to reset the theta controller to the current pose's
        // heading.
        if (firstRun) {
            thetaController.reset(currentPose.getRotation().getRadians());
            xController.reset();
            yController.reset();
            firstRun = false;
        }
    
        // Calculate feedforward velocities (field-relative).
        double xFF = linearVelocityRefMeters * directionOfMovement.getCos();
        double yFF = linearVelocityRefMeters * directionOfMovement.getSin();
        // Theta seems to need to be inverted (at least for sim)
        double thetaFF =
            thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());
        
        if (smartDashboardEnabled) {
            SmartDashboard.putNumber("RotatingHolonomicDriveController/currentRadians", currentPose.getRotation().getRadians());
            SmartDashboard.putNumber("RotatingHolonomicDriveController/targetRadians", angleRef.getRadians());
            SmartDashboard.putNumber("RotatingHolonomicDriveController/thetaFF", thetaFF);

            SmartDashboard.putNumber("RotatingHolonomicDriveController/currentX", currentPose.getX());
            SmartDashboard.putNumber("RotatingHolonomicDriveController/targetX", poseRef.getX());
            SmartDashboard.putNumber("RotatingHolonomicDriveController/xFF", xFF);

            SmartDashboard.putNumber("RotatingHolonomicDriveController/currentY", currentPose.getY());
            SmartDashboard.putNumber("RotatingHolonomicDriveController/targetY", poseRef.getY());
            SmartDashboard.putNumber("RotatingHolonomicDriveController/yFF", yFF);
        }
    
        poseError = poseRef.relativeTo(currentPose);
        rotationError = angleRef.minus(currentPose.getRotation());
    
        if (!enabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
        }
    
        // Calculate feedback velocities (based on position error).
        double xFeedback = xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = yController.calculate(currentPose.getY(), poseRef.getY());
    
        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
    }
  
    /**
     * Returns the next output of the holonomic drive controller.
     *
     * @param currentPose The current pose.
     * @param desiredState The desired trajectory state.
     * @return The next output of the holonomic drive controller.
     */
    public ChassisSpeeds calculate(
            Pose2d currentPose, HolonomicTrajectory.State desiredState) {
        return calculate(
            currentPose, desiredState.poseMeters, desiredState.directionOfMovement, desiredState.velocityMetersPerSecond, desiredState.poseMeters.getRotation());
    }
  
    /**
     * Enables and disables the controller for troubleshooting problems. When calculate() is called on
     * a disabled controller, only feedforward values are returned.
     *
     * @param enabled If the controller is enabled or not.
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Makes the controller run as if it's running for the first time.
     */
    public void reset() {
        firstRun = true;
    }
}
