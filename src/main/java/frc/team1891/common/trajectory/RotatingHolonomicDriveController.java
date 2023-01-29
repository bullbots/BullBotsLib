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
public class RotatingHolonomicDriveController {
    public static boolean smartDashboardEnabled = false;
    /**
     * SmartDashboard values for debug.
     * @param enable show or hide SmartDashboard values
     */
    public static void enableSmartDashboard(boolean enable) {
        smartDashboardEnabled = enable;
    }

    private Pose2d m_poseError = new Pose2d();
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private boolean m_enabled = true;
  
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;
  
    private boolean m_firstRun = true;
  
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
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
    }
  
    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        final var eTranslate = m_poseError.getTranslation();
        final var eRotate = m_rotationError;
        final var tolTranslate = m_poseTolerance.getTranslation();
        final var tolRotate = m_poseTolerance.getRotation();
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
        m_poseTolerance = tolerance;
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
        if (m_firstRun) {
            m_thetaController.reset(currentPose.getRotation().getRadians());
            m_firstRun = false;
        }
    
        // Calculate feedforward velocities (field-relative).
        double xFF = linearVelocityRefMeters * directionOfMovement.getCos();
        double yFF = linearVelocityRefMeters * directionOfMovement.getSin();
        // Theta seems to need to be inverted (at least for sim)
        double thetaFF =
            m_thetaController.calculate(currentPose.getRotation().getRadians(), angleRef.getRadians());
        
        if (smartDashboardEnabled) {
            SmartDashboard.putNumber("currentRadians", currentPose.getRotation().getRadians());
            SmartDashboard.putNumber("targetRadians", angleRef.getRadians());
            SmartDashboard.putNumber("thetaFF", thetaFF);

            SmartDashboard.putNumber("currentX", currentPose.getX());
            SmartDashboard.putNumber("targetX", poseRef.getX());
            SmartDashboard.putNumber("xFF", xFF);

            SmartDashboard.putNumber("currentY", currentPose.getY());
            SmartDashboard.putNumber("targetY", poseRef.getY());
            SmartDashboard.putNumber("yFF", yFF);
        }
    
        m_poseError = poseRef.relativeTo(currentPose);
        m_rotationError = angleRef.minus(currentPose.getRotation());
    
        if (!m_enabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
        }
    
        // Calculate feedback velocities (based on position error).
        double xFeedback = m_xController.calculate(currentPose.getX(), poseRef.getX());
        double yFeedback = m_yController.calculate(currentPose.getY(), poseRef.getY());
    
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
        m_enabled = enabled;
    }

    /**
     * Makes the controller run as if it's running for the first time.
     */
    public void reset() {
        m_firstRun = true;
    }
}
