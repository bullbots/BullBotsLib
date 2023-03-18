package frc.team1891.common.can;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

class CANMessageGeneratorTest {
    @Test
    void testCANBinaries() {
        System.out.println("LuminaryMicro - Jaguar Motor Controller - Speed Control Mode Disabled: "+CANMessageGenerator.generateCANMessageBinary(
                CANDeviceType.MotorController,
                CANManufacturer.LuminaryMicro,
                CANAPI.JaguarMotorControllerClass.SpeedControlMode,
                CANAPI.JaguarMotorControllerIndex.EnableControl,
                4
        ));

        // See https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html#device-number
        Assertions.assertEquals("0b00010000000100000010001000100", CANMessageGenerator.generateCANMessageBinary(
                CANDeviceType.MotorController,
                CANManufacturer.LuminaryMicro,
                CANAPI.JaguarMotorControllerClass.SpeedControlMode,
                CANAPI.JaguarMotorControllerIndex.DisableControl,
                4
        ));

        // TODO: I don't think any of these are correct

        System.out.println("Broadcast Disable: "+CANMessageGenerator.generateCANBroadcastMessageBinary(
                CANBroadcastMessage.Disable
        ));

        // See https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html#device-number
        Assertions.assertEquals("0b00000000000000000000000000000", CANMessageGenerator.generateCANBroadcastMessageBinary(
                CANBroadcastMessage.Disable
        ));

        System.out.println("Broadcast Heartbeat: "+CANMessageGenerator.generateCANBroadcastMessageBinary(
                CANBroadcastMessage.Heartbeat
        ));

        // See https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html#device-number
        Assertions.assertEquals("0b00000000000000000000000000101", CANMessageGenerator.generateCANBroadcastMessageBinary(
                CANBroadcastMessage.Heartbeat
        ));
    }
}