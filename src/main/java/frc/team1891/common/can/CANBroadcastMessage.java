package frc.team1891.common.can;

public enum CANBroadcastMessage {
    Disable(0),
    SystemHalt(1),
    SystemReset(2),
    DeviceAssign(3),
    DeviceQuery(4),
    Heartbeat(5),
    Sync(6),
    Update(7),
    FirmwareVersion(8),
    Enumerate(9),
    SystemResume(10);

    public final int value;
    CANBroadcastMessage(int value) {
        this.value = value;
    }

    // TODO: I don't know the actual length
    public static final int SEGMENT_LENGTH = 5;
    public String getBinary() {
        String binary = Integer.toBinaryString(value);
        if (binary.length() < SEGMENT_LENGTH) {
            binary = "0".repeat(SEGMENT_LENGTH-binary.length()) + binary;
        }
        return binary;
    }
}
