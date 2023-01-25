package frc.team1891.common.drivetrains.sim;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.team1891.common.drivetrains.DrivetrainConfig;

public class SwerveSim {
    private final DrivetrainConfig config;
    private final TalonFXSimCollection frontLeftDriveSim;
    private final WPI_TalonFX frontLeftDrive;
    private final TalonFXSimCollection frontLeftSteerSim;
    private final WPI_TalonFX frontLeftSteer;
    private final CANCoderSimCollection frontLeftEncoder;
    private final TalonFXSimCollection frontRightDriveSim;
    private final WPI_TalonFX frontRightDrive;
    private final TalonFXSimCollection frontRightSteerSim;
    private final WPI_TalonFX frontRightSteer;
    private final CANCoderSimCollection frontRightEncoder;
    private final TalonFXSimCollection backLeftDriveSim;
    private final WPI_TalonFX backLeftDrive;
    private final TalonFXSimCollection backLeftSteerSim;
    private final WPI_TalonFX backLeftSteer;
    private final CANCoderSimCollection backLeftEncoder;
    private final TalonFXSimCollection backRightDriveSim;
    private final WPI_TalonFX backRightDrive;
    private final TalonFXSimCollection backRightSteerSim;
    private final WPI_TalonFX backRightSteer;
    private final CANCoderSimCollection backRightEncoder;

    public SwerveSim(
            DrivetrainConfig config,
            WPI_TalonFX frontLeftDrive,
            WPI_TalonFX frontLeftSteer,
            WPI_CANCoder frontLeftEncoder,
            WPI_TalonFX frontRightDrive,
            WPI_TalonFX frontRightSteer,
            WPI_CANCoder frontRightEncoder,
            WPI_TalonFX backLeftDrive,
            WPI_TalonFX backLeftSteer,
            WPI_CANCoder backLeftEncoder,
            WPI_TalonFX backRightDrive,
            WPI_TalonFX backRightSteer,
            WPI_CANCoder backRightEncoder
    ) {
        this.config = config;
        this.frontLeftDrive = frontLeftDrive;
        this.frontLeftDriveSim = frontLeftDrive.getSimCollection();
        this.frontLeftSteer = frontLeftSteer;
        this.frontLeftSteerSim = frontLeftSteer.getSimCollection();
        this.frontLeftEncoder = frontLeftEncoder.getSimCollection();
        this.frontRightDrive = frontRightDrive;
        this.frontRightDriveSim = frontRightDrive.getSimCollection();
        this.frontRightSteer = frontRightSteer;
        this.frontRightSteerSim = frontRightSteer.getSimCollection();
        this.frontRightEncoder = frontRightEncoder.getSimCollection();
        this.backLeftDrive = backLeftDrive;
        this.backLeftDriveSim = backLeftDrive.getSimCollection();
        this.backLeftSteer = backLeftSteer;
        this.backLeftSteerSim = backLeftSteer.getSimCollection();
        this.backLeftEncoder = backLeftEncoder.getSimCollection();
        this.backRightDrive = backRightDrive;
        this.backRightDriveSim = backRightDrive.getSimCollection();
        this.backRightSteer = backRightSteer;
        this.backRightSteerSim = backRightSteer.getSimCollection();
        this.backRightEncoder = backRightEncoder.getSimCollection();
    }

    public void update() {
        frontLeftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
        frontLeftDriveSim.setIntegratedSensorRawPosition((int)
                (frontLeftDrive.getSelectedSensorPosition() +
                        talonPercentToEncoderTicksPerSecond(frontLeftDrive.getMotorOutputPercent()) * .02 // 20 ms, roughly how often it'll be updated
                ));
        frontLeftDriveSim.setIntegratedSensorVelocity((int)
                talonPercentToEncoderTicksPerSecond(frontLeftDrive.getMotorOutputPercent()) / 10
        );
        frontLeftSteerSim.setBusVoltage(RobotController.getBatteryVoltage());
        frontLeftSteerSim.setIntegratedSensorRawPosition((int)
                (frontLeftSteer.getSelectedSensorPosition() +
                        talonPercentToEncoderTicksPerSecond(frontLeftSteer.getMotorOutputPercent()) * .02 // 20 ms, roughly how often it'll be updated
        ));
        frontLeftSteerSim.setIntegratedSensorVelocity((int)
                talonPercentToEncoderTicksPerSecond(frontLeftSteer.getMotorOutputPercent()) / 10
        );
        frontLeftEncoder.setRawPosition((int) talonEncoderTicksToCANCoderPosition(frontLeftSteer.getSelectedSensorPosition()));
        frontRightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
        frontRightDriveSim.setIntegratedSensorRawPosition((int)
                (frontRightDrive.getSelectedSensorPosition() +
                        talonPercentToEncoderTicksPerSecond(frontRightDrive.getMotorOutputPercent()) * .02 // 20 ms, roughly how often it'll be updated
        ));
        frontRightDriveSim.setIntegratedSensorVelocity((int)
                talonPercentToEncoderTicksPerSecond(frontRightDrive.getMotorOutputPercent()) / 10
        );
        frontRightSteerSim.setBusVoltage(RobotController.getBatteryVoltage());
        frontRightSteerSim.setIntegratedSensorRawPosition((int)
                (frontRightSteer.getSelectedSensorPosition() +
                        talonPercentToEncoderTicksPerSecond(frontRightSteer.getMotorOutputPercent()) * .02 // 20 ms, roughly how often it'll be updated
        ));
        frontRightSteerSim.setIntegratedSensorVelocity((int)
                talonPercentToEncoderTicksPerSecond(frontRightSteer.getMotorOutputPercent()) / 10
        );
        frontRightEncoder.setRawPosition((int) talonEncoderTicksToCANCoderPosition(frontRightSteer.getSelectedSensorPosition()));
        backLeftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
        backLeftDriveSim.setIntegratedSensorRawPosition((int)
                (backLeftDrive.getSelectedSensorPosition() +
                        talonPercentToEncoderTicksPerSecond(backLeftDrive.getMotorOutputPercent()) * .02 // 20 ms, roughly how often it'll be updated
        ));
        backLeftDriveSim.setIntegratedSensorVelocity((int)
                talonPercentToEncoderTicksPerSecond(backLeftDrive.getMotorOutputPercent()) / 10
        );
        backLeftSteerSim.setBusVoltage(RobotController.getBatteryVoltage());
        backLeftSteerSim.setIntegratedSensorRawPosition((int)
                (backLeftSteer.getSelectedSensorPosition() +
                        talonPercentToEncoderTicksPerSecond(backLeftSteer.getMotorOutputPercent()) * .02 // 20 ms, roughly how often it'll be updated
        ));
        backLeftSteerSim.setIntegratedSensorVelocity((int)
                talonPercentToEncoderTicksPerSecond(backLeftSteer.getMotorOutputPercent()) / 10
        );
        backLeftEncoder.setRawPosition((int) talonEncoderTicksToCANCoderPosition(backLeftSteer.getSelectedSensorPosition()));
        backRightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
        backRightDriveSim.setIntegratedSensorRawPosition((int)
                (backRightDrive.getSelectedSensorPosition() +
                        talonPercentToEncoderTicksPerSecond(backRightDrive.getMotorOutputPercent()) * .02 // 20 ms, roughly how often it'll be updated
        ));
        backRightDriveSim.setIntegratedSensorVelocity((int)
                talonPercentToEncoderTicksPerSecond(backRightDrive.getMotorOutputPercent()) / 10
        );
        backRightSteerSim.setBusVoltage(RobotController.getBatteryVoltage());
        backRightSteerSim.setIntegratedSensorRawPosition((int)
                (backRightSteer.getSelectedSensorPosition() +
                        talonPercentToEncoderTicksPerSecond(backRightSteer.getMotorOutputPercent()) * .02 // 20 ms, roughly how often it'll be updated
        ));
        backRightSteerSim.setIntegratedSensorVelocity((int)
                talonPercentToEncoderTicksPerSecond(backRightSteer.getMotorOutputPercent()) / 10
        );
        backRightEncoder.setRawPosition((int) talonEncoderTicksToCANCoderPosition(backRightSteer.getSelectedSensorPosition()));
    }

    private double talonPercentToEncoderTicksPerSecond(double percent) {
        // Max freespin RPM / 60 Seconds per minute / gear ratio * 2024 Encoder ticks per revolution
        return percent * 6380 / 60d  / config.gearRatio * 2048;
    }

    private double talonEncoderTicksToCANCoderPosition(double ticks) {
        // 1 / 2048 Encoder ticks per revolution / steering gear ratio of mk4i * 360 degrees per revolution
        return ticks / 2048 / (150/7d) * 360;
    }
}
