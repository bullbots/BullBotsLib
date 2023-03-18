package frc.team1891.common.can;

/**
 * See https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
 */
public enum CANDeviceType {
    BroadcastMessages(0),
    RobotController(1),
    MotorController(2),
    RelayController(3),
    GyroSensor(4),
    Accelerometer(5),
    UltrasonicSensor(6),
    GearToothSensor(7),
    PowerDistributionModule(8),
    PneumaticsController(9),
    Miscellaneous(10),
    IOBreakout(11),
    /*Reserved 12-30*/
    FirmwareUpdate(31);

    private final int value;
    CANDeviceType(int value) {
        this.value = value;
    }

    public static final int SEGMENT_LENGTH = 5;
    public String getBinary() {
        String binary = Integer.toBinaryString(value);
        if (binary.length() < SEGMENT_LENGTH) {
            binary = "0".repeat(SEGMENT_LENGTH-binary.length()) + binary;
        }
        return binary;
    }
}
