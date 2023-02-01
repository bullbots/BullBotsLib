package frc.team1891.common.drivetrains;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class bsfSwerveModule {
    private DrivetrainConfig config;
    private Rotation2d lastAngle;

    private SwerveModuleState desiredState;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double encoderOffset;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(.75, 0, 0);

    public bsfSwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX steerMotor, CANCoder encoder, double encoderOffset, DrivetrainConfig config){
        this.config = config;

        /* Angle Encoder Config */
        angleEncoder = encoder;
        configAngleEncoder();

        this.encoderOffset = encoderOffset;

        /* Angle Motor Config */
        mAngleMotor = steerMotor;
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = driveMotor;
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        this.desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        setSpeed(this.desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / config.chassisMaxVelocityMetersPerSecond;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, 2*Math.PI*config.wheelRadiusMeters, config.gearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (config.chassisMaxVelocityMetersPerSecond * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), 150/7.));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), 150/7.));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - encoderOffset, 150/7.);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){
        angleEncoder.configFactoryDefault();
        CANCoderConfiguration configuration = new CANCoderConfiguration();
        configuration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        configuration.sensorDirection = false;
        configuration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        angleEncoder.configAllSettings(configuration);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                true,
                25,
                40,
                .1);

        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.slot0.kP = 1;
        configuration.slot0.kI = 0;
        configuration.slot0.kD = 0;
        configuration.slot0.kF = 0;
        configuration.supplyCurrLimit = angleSupplyLimit;
        mAngleMotor.configAllSettings(configuration);
        mAngleMotor.setInverted(false);
        mAngleMotor.setNeutralMode(NeutralMode.Coast);
        resetToAbsolute();
    }

    private void configDriveMotor(){
        mDriveMotor.configFactoryDefault();
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                true,
                35,
                60,
                .1
        );

        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.slot0.kP = 1;
        configuration.slot0.kI = 0;
        configuration.slot0.kD = 0;
        configuration.slot0.kF = 0;
        configuration.supplyCurrLimit = driveSupplyLimit;
        configuration.openloopRamp = .25;
        configuration.closedloopRamp = 0;
        mDriveMotor.configAllSettings(configuration);
        mDriveMotor.setInverted(false);
        mDriveMotor.setNeutralMode(NeutralMode.Brake);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
                Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), 2*Math.PI*config.wheelRadiusMeters, config.gearRatio),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), 2*Math.PI*config.wheelRadiusMeters, config.gearRatio),
                getAngle()
        );
    }

    /**
     * Configure the module to output it's information to Shuffleboard
     * @param moduleLayout the layout for the module
     */
    public void configureShuffleboard(ShuffleboardLayout moduleLayout) {
        moduleLayout.addNumber("Current Position (Meters)", () -> getPosition().distanceMeters);
        moduleLayout.addNumber("Current Velocity (Meters per Second)", () -> getState().speedMetersPerSecond);
        moduleLayout.addNumber("Current Angle (Radians)", () -> getAngle().getRadians());
        moduleLayout.addNumber("Desired Velocity (Meters per Second)", () -> desiredState.speedMetersPerSecond);
        moduleLayout.addNumber("Desired Angle (Radians)", () -> desiredState.angle.getRadians());
    }

    public static class Conversions {
        /**
         * @param positionCounts CANCoder Position Counts
         * @param gearRatio Gear Ratio between CANCoder and Mechanism
         * @return Degrees of Rotation of Mechanism
         */
        public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
            return positionCounts * (360.0 / (gearRatio * 4096.0));
        }

        /**
         * @param degrees Degrees of rotation of Mechanism
         * @param gearRatio Gear Ratio between CANCoder and Mechanism
         * @return CANCoder Position Counts
         */
        public static double degreesToCANcoder(double degrees, double gearRatio) {
            return degrees / (360.0 / (gearRatio * 4096.0));
        }

        /**
         * @param positionCounts Falcon Position Counts
         * @param gearRatio Gear Ratio between Falcon and Mechanism
         * @return Degrees of Rotation of Mechanism
         */
        public static double falconToDegrees(double positionCounts, double gearRatio) {
            return positionCounts * (360.0 / (gearRatio * 2048.0));
        }

        /**
         * @param degrees Degrees of rotation of Mechanism
         * @param gearRatio Gear Ratio between Falcon and Mechanism
         * @return Falcon Position Counts
         */
        public static double degreesToFalcon(double degrees, double gearRatio) {
            return degrees / (360.0 / (gearRatio * 2048.0));
        }

        /**
         * @param velocityCounts Falcon Velocity Counts
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
         * @return RPM of Mechanism
         */
        public static double falconToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * (600.0 / 2048.0);
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        /**
         * @param RPM RPM of mechanism
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
         * @return RPM of Mechanism
         */
        public static double RPMToFalcon(double RPM, double gearRatio) {
            double motorRPM = RPM * gearRatio;
            double sensorCounts = motorRPM * (2048.0 / 600.0);
            return sensorCounts;
        }

        /**
         * @param velocitycounts Falcon Velocity Counts
         * @param circumference Circumference of Wheel
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
         * @return Falcon Velocity Counts
         */
        public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
            double wheelRPM = falconToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / 60;
            return wheelMPS;
        }

        /**
         * @param velocity Velocity MPS
         * @param circumference Circumference of Wheel
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
         * @return Falcon Velocity Counts
         */
        public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
            double wheelRPM = ((velocity * 60) / circumference);
            double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
            return wheelVelocity;
        }

        /**
         * @param positionCounts Falcon Position Counts
         * @param circumference Circumference of Wheel
         * @param gearRatio Gear Ratio between Falcon and Wheel
         * @return Meters
         */
        public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
            return positionCounts * (circumference / (gearRatio * 2048.0));
        }

        /**
         * @param meters Meters
         * @param circumference Circumference of Wheel
         * @param gearRatio Gear Ratio between Falcon and Wheel
         * @return Falcon Position Counts
         */
        public static double MetersToFalcon(double meters, double circumference, double gearRatio){
            return meters / (circumference / (gearRatio * 2048.0));
        }
    }
}
