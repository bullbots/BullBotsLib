package frc.team1891.common.can;

public class CANMessageGenerator {
    private CANMessageGenerator() {}

    public static String generateCANMessageBinary(
            CANDeviceType deviceType,
            CANManufacturer manufacturer,
            CANAPI.APIClass apiClass,
            CANAPI.APIIndex apiIndex,
            int deviceNumber
    ) {
        String deviceTypeBinary = deviceType.getBinary();
        String manufacturerBinary = manufacturer.getBinary();
        String apiClassBinary = apiClass.getBinary();
        String apiIndexBinary = apiIndex.getBinary();

        String deviceNumberBinary = Integer.toBinaryString(deviceNumber);
        if (deviceNumberBinary.length() < 6) {
            deviceNumberBinary = "0".repeat(6-deviceNumberBinary.length()) + deviceNumberBinary;
        }

        String binaryMessage =
                deviceTypeBinary +
                manufacturerBinary +
                apiClassBinary +
                apiIndexBinary +
                deviceNumberBinary;

        return "0b"+binaryMessage;
    }

    public static int generateCANMessage(
            CANDeviceType deviceType,
            CANManufacturer manufacturer,
            CANAPI.APIClass apiClass,
            CANAPI.APIIndex apiIndex,
            int deviceNumber
    ) {
        return Integer.parseInt(generateCANMessageBinary(
                deviceType,
                manufacturer,
                apiClass,
                apiIndex,
                deviceNumber
        ).substring(2), 2);
    }

    public static String generateCANBroadcastMessageBinary(CANBroadcastMessage broadcastMessage) {
        return generateCANMessageBinary(
                CANDeviceType.BroadcastMessages,
                CANManufacturer.Broadcast,
                CANAPI.APIClassBroadcast.Broadcast,
                CANAPI.APIIndexBroadcast.Broadcast,
                broadcastMessage.value
                );
    }

    public static int generateCANBroadcastMessage(CANBroadcastMessage broadcastMessage) {
        return Integer.parseInt(generateCANBroadcastMessageBinary(broadcastMessage).substring(2), 2);
    }
}
