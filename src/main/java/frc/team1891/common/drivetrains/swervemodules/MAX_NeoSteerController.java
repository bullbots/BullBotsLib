package frc.team1891.common.drivetrains.swervemodules;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * https://github.com/REVrobotics/MAXSwerve-Java-Template
 */
public class MAX_NeoSteerController implements SteerController {
    private final CANSparkMax m_turningNeo;

    private final AbsoluteEncoder m_turningEncoder;

    private final SparkMaxPIDController m_turningPIDController;

    private double m_chassisAngularOffsetRadians = 0;

    /**
     * Constructs a MAX_NeoSteerController and configures the turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAX_NeoSteerController(int turningCANId, double chassisAngularOffset, double kP, double kI, double kD, double kFF) {
        this(new CANSparkMax(turningCANId, CANSparkMaxLowLevel.MotorType.kBrushless), chassisAngularOffset, kP, kI, kD, kFF);
    }
    /**
     * Constructs a MAX_NeoSteerController and configures the turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAX_NeoSteerController(CANSparkMax turningNeo, double chassisAngularOffsetRadians, double kP, double kI, double kD, double kFF) {
        m_turningNeo = turningNeo;

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        m_turningNeo.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the turning SPARKS MAX.
        m_turningEncoder = m_turningNeo.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        m_turningPIDController = m_turningNeo.getPIDController();
        m_turningPIDController.setFeedbackDevice(m_turningEncoder);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        m_turningEncoder.setPositionConversionFactor((2 * Math.PI));
        m_turningEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.);

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        m_turningEncoder.setInverted(true);

        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        m_turningPIDController.setPositionPIDWrappingEnabled(true);
        m_turningPIDController.setPositionPIDWrappingMinInput(0);
        m_turningPIDController.setPositionPIDWrappingMaxInput((2 * Math.PI)); // Match positionConversionFactor

        m_turningPIDController.setP(kP);
        m_turningPIDController.setI(kI);
        m_turningPIDController.setD(kD);
        m_turningPIDController.setFF(kFF);
        m_turningPIDController.setOutputRange(-1, 1);

        m_turningNeo.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_turningNeo.setSmartCurrentLimit(20);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        m_turningNeo.burnFlash();

        m_chassisAngularOffsetRadians = chassisAngularOffsetRadians;
    }

    @Override
    public void drive(SwerveModuleState state) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
        correctedDesiredState.angle = state.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffsetRadians));

        //TODO: state may need re-optimized here

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                new Rotation2d(m_turningEncoder.getPosition()));

        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void stop() {
        m_turningNeo.stopMotor();
    }

    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffsetRadians);
    }

    @Override
    public double getRadians() {
        return m_turningEncoder.getPosition() - m_chassisAngularOffsetRadians;
    }

    @Override
    public double getDegrees() {
        return getRotation2d().getDegrees();
    }
}
