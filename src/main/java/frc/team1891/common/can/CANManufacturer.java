package frc.team1891.common.can;

public enum CANManufacturer {
    Broadcast(0),
    NI(1),
    LuminaryMicro(2),
    DEKA(3),
    CTRElectronics(4),
    REVRobotics(5),
    Grapple(6),
    MindSensors(7),
    TeamUse(8),
    KauaiLabs(9),
    Copperforge(10),
    PlayingWithFusion(11),
    Studica(12),
    TheThriftyBot(13),
    /*Reserved 14-255*/;

    private final int value;
    CANManufacturer(int value) {
        this.value = value;
    }

    public static final int SEGMENT_LENGTH = 8;
    public String getBinary() {
        String binary = Integer.toBinaryString(value);
        if (binary.length() < SEGMENT_LENGTH) {
            binary = "0".repeat(SEGMENT_LENGTH-binary.length()) + binary;
        }
        return binary;    }
}
