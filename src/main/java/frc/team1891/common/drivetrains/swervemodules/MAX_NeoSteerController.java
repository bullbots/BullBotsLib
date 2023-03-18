package frc.team1891.common.drivetrains.swervemodules;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team1891.common.hardware.WPI_CANSparkMax;

/**
 * https://github.com/REVrobotics/MAXSwerve-Java-Template
 */
@SuppressWarnings("unused")
public class MAX_NeoSteerController implements SteerController {
    protected final WPI_CANSparkMax turningNeo;

    protected final AbsoluteEncoder turningEncoder;

    protected final SparkMaxPIDController turningPIDController;

    protected double chassisAngularOffsetRadians;

    /**
     * Constructs a MAX_NeoSteerController and configures the turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAX_NeoSteerController(int turningMotorCANId, double chassisAngularOffset, double kP, double kI, double kD, double kFF) {
        this(new WPI_CANSparkMax(turningMotorCANId, CANSparkMaxLowLevel.MotorType.kBrushless), chassisAngularOffset, kP, kI, kD, kFF);
    }
    /**
     * Constructs a MAX_NeoSteerController and configures the turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAX_NeoSteerController(WPI_CANSparkMax turningNeo, double chassisAngularOffsetRadians, double kP, double kI, double kD, double kFF) {
        this.turningNeo = turningNeo;

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        this.turningNeo.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the turning SPARKS MAX.
        turningEncoder = this.turningNeo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        turningPIDController = this.turningNeo.getPIDController();
        turningPIDController.setFeedbackDevice(turningEncoder);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        // The native units of the encoder is rotations
        turningEncoder.setPositionConversionFactor((2 * Math.PI));
        turningEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        turningEncoder.setInverted(true);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(0);
        turningPIDController.setPositionPIDWrappingMaxInput((2 * Math.PI)); // Match positionConversionFactor

        turningPIDController.setP(kP);
        turningPIDController.setI(kI);
        turningPIDController.setD(kD);
        turningPIDController.setFF(kFF);
        turningPIDController.setOutputRange(-1, 1);

        this.turningNeo.setIdleMode(WPI_CANSparkMax.IdleMode.kBrake);
        this.turningNeo.setSmartCurrentLimit(20);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        this.turningNeo.burnFlash();

        this.chassisAngularOffsetRadians = chassisAngularOffsetRadians;
    }

    @Override
    public void drive(SwerveModuleState state) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedDesiredState.angle = state.angle.plus(Rotation2d.fromRadians(chassisAngularOffsetRadians));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(turningEncoder.getPosition()));

        turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), WPI_CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void stop() {
        turningNeo.stopMotor();
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(turningEncoder.getPosition() - chassisAngularOffsetRadians);
    }

    @Override
    public double getRadians() {
        return turningEncoder.getPosition() - chassisAngularOffsetRadians;
    }

    @Override
    public double getDegrees() {
        return getRotation2d().getDegrees();
    }
}
