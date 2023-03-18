package frc.team1891.common.can;

public class CANAPI {
    private CANAPI() {}

    public interface APIClass {
        int SEGMENT_LENGTH = 6;
        String getBinary();
    }

    private static final int INDEX_SEGMENT_LENGTH = 4;
    public interface APIIndex {
        int SEGMENT_LENGTH = 4;
        String getBinary();
    }

    public enum APIClassBroadcast implements APIClass {
        Broadcast(0);

        private final int value;
        APIClassBroadcast(int value) {
            this.value = value;
        }

        @Override
        public String getBinary() {
            String binary = Integer.toBinaryString(value);
            if (binary.length() < SEGMENT_LENGTH) {
                binary = "0".repeat(SEGMENT_LENGTH -binary.length()) + binary;
            }
            return binary;
        }
    }

    public enum APIIndexBroadcast implements APIIndex {
        Broadcast(0);

        private final int value;
        APIIndexBroadcast(int value) {
            this.value = value;
        }

        @Override
        public String getBinary() {
            String binary = Integer.toBinaryString(value);
            if (binary.length() < SEGMENT_LENGTH) {
                binary = "0".repeat(SEGMENT_LENGTH -binary.length()) + binary;
            }
            return binary;
        }
    }

    public enum JaguarMotorControllerClass implements APIClass {
        VoltageControlMode(0),
        SpeedControlMode(1),
        VoltageCompensationMode(2),
        PositionControlMode(3),
        CurrentControlMode(4),
        Status(5),
        PeriodicStatus(6),
        Configuration(7),
        Ack(8);

        private final int value;
        JaguarMotorControllerClass(int value) {
            this.value = value;
        }

        @Override
        public String getBinary() {
            String binary = Integer.toBinaryString(value);
            if (binary.length() < SEGMENT_LENGTH) {
                binary = "0".repeat(SEGMENT_LENGTH -binary.length()) + binary;
            }
            return binary;
        }
    }

    public enum JaguarMotorControllerIndex implements APIIndex {
        EnableControl(0),
        DisableControl(1),
        SetSetpoint(2),
        PConstant(3),
        IConstant(4),
        DConstant(5),
        SetReference(6),
        TrustedEnabled(7),
        TrustedSetNoAck(8),
        TrustedSetSetpointNoAck(10),
        SetSetpointNoAck(11);

        private final int value;
        JaguarMotorControllerIndex(int value) {
            this.value = value;
        }

        @Override
        public String getBinary() {
            String binary = Integer.toBinaryString(value);
            if (binary.length() < SEGMENT_LENGTH) {
                binary = "0".repeat(SEGMENT_LENGTH -binary.length()) + binary;
            }
            return binary;
        }
    }
}
