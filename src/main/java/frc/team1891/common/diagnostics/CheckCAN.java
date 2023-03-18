package frc.team1891.common.diagnostics;

import edu.wpi.first.math.Pair;
import frc.team1891.common.can.CANDeviceFinder;

import java.util.List;

@SuppressWarnings("unused")
public class CheckCAN {
    private CheckCAN() {}

    /**
     * Returns true if all the devices are found, and false if one or more devices are missing.
     * This should be run on a separate thread!
     * @param expectedDevices the devices expected to be on the CAN bus
     * @return if the CAN bus contains all the expected devices
     */
    public static boolean check(List<Pair<Integer, String>> expectedDevices) {
        CANDeviceFinder.find();
        for (Pair<Integer, String> expectedDevice : expectedDevices) {
            int deviceID = expectedDevice.getFirst();
            String device = expectedDevice.getSecond();
            switch (device) {
                case "PDP":
                    if (!CANDeviceFinder.isPDPPresent()) {
                        return false;
                    }
                    break;
                case "PDH":
                    if (!CANDeviceFinder.isDevicePresent("PDH")) {
                        return false;
                    }
                    break;
                case "PCM":
                    if (!CANDeviceFinder.isPCMPresent(deviceID)) {
                        return false;
                    }
                    break;
                case "TalonSRX":
                    if (!CANDeviceFinder.isSRXPresent(deviceID)) {
                        return false;
                    }
                    break;
                    // TODO: write method for TalonFX
                case "TalonFX":
                    if (!CANDeviceFinder.isDevicePresent("FX")) {
                        return false;
                    }
                    break;
                case "VictorSPX":
                    if (!CANDeviceFinder.isSPXPresent(deviceID)) {
                        return false;
                    }
                    break;
                case "SparkMAX":
                    if (!CANDeviceFinder.isMAXPresent(deviceID)) {
                        return false;
                    }
                    break;
                default:
                    System.err.printf("Device, \"%s\", at index %d is not a recognized CAN Device.  This doesn't " +
                            "necessarily mean the device isn't on the CAN bus.", device, deviceID);
                    return false;
            }
        }
        return true;
    }

    public static List<String> getDetectedDevices() {
        CANDeviceFinder.find();
        return CANDeviceFinder.getDeviceList();
    }
}
