package frc.team1891.common.control;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Want to be the coolest team at competition? Here's your controller.
 * https://www.logitechg.com/en-au/products/space/x52-pro-space-flight-simulator-controller.945-000022.html
 */
@SuppressWarnings("unused")
public class X52ProfessionalHOTAS extends GenericHID {
    public enum Axis {
        // TODO: There are like 7 axis
        JoystickX(0),
        JoystickY(1),
        Throttle(2);

        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public enum Button {
        JoystickTriggerFirstLevel(1),
        Fire(2),
        A(3),
        B(4),
        C(5),
        Joystick4thFingerTrigger(6),
        D(7),
        E(8),
        T1Up(9),
        T1Down(10),
        T2Up(11),
        T2Down(12),
        T3Up(13),
        T3Down(14),
        // TODO: Unkown
        Unknown15(15),
        MouseButton(16),
        ScrollDownThrottle2ndFinger(17),
        ScrollUpThrottle2ndFinger(18),
        ScrollPressThrottle2ndFinger(19),
        JoystickBlackPOVUp(20),
        JoystickBlackPOVRight(21),
        JoystickBlackPOVDown(22),
        JoystickBlackPOVLeft(23),
        Throttle1stFingerPOVBack(24),
        Throttle1stFingerPOVLeft(25),
        Throttle1stFingerPOVForward(26),
        Throttle1stFingerPOVRight(27),
        // TODO: Unkown
        Unknown28(28),
        Unknown29(29),
        Unknown30(30),
        Unknown31(31),
        Unknown32(32);

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    /**
     * Construct an instance of a device.
     *
     * @param port The port index on the Driver Station that the device is plugged into.
     */
    public X52ProfessionalHOTAS(int port) {
        super(port);

        HAL.report(tResourceType.kResourceType_Joystick, port + 1);
    }

    public double getJoystickX() {
        return getRawAxis(Axis.JoystickX.value);
    }

    public double getJoystickY() {
        return getRawAxis(Axis.JoystickY.value);
    }

    public double getThrottle() {
        return getRawAxis(Axis.Throttle.value);
    }

    // TODO: Finish axis

    public boolean getJoystickTriggerFirstLevel() {
        return getRawButton(Button.JoystickTriggerFirstLevel.value);
    }

    public boolean getJoystickTriggerFirstLevelPressed() {
        return getRawButtonPressed(Button.JoystickTriggerFirstLevel.value);
    }

    public boolean getJoystickTriggerFirstLevelReleased() {
        return getRawButtonReleased(Button.JoystickTriggerFirstLevel.value);
    }

    public BooleanEvent joystickTriggerfirstLevel(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickTriggerFirstLevel);
    }

    public boolean getFire() {
        return getRawButton(Button.Fire.value);
    }

    public boolean getFirePressed() {
        return getRawButtonPressed(Button.Fire.value);
    }

    public boolean getFireReleased() {
        return getRawButtonReleased(Button.Fire.value);
    }

    public BooleanEvent fire(EventLoop loop) {
        return new BooleanEvent(loop, this::getFire);
    }

    public boolean getA() {
        return getRawButton(Button.A.value);
    }

    public boolean getAPressed() {
        return getRawButtonPressed(Button.A.value);
    }

    public boolean getAReleased() {
        return getRawButtonReleased(Button.A.value);
    }

    public BooleanEvent a(EventLoop loop) {
        return new BooleanEvent(loop, this::getA);
    }

    public boolean getB() {
        return getRawButton(Button.B.value);
    }

    public boolean getBPressed() {
        return getRawButtonPressed(Button.B.value);
    }

    public boolean getBReleased() {
        return getRawButtonReleased(Button.B.value);
    }

    public BooleanEvent b(EventLoop loop) {
        return new BooleanEvent(loop, this::getB);
    }

    public boolean getC() {
        return getRawButton(Button.C.value);
    }

    public boolean getCPressed() {
        return getRawButtonPressed(Button.C.value);
    }

    public boolean getCReleased() {
        return getRawButtonReleased(Button.C.value);
    }

    public BooleanEvent c(EventLoop loop) {
        return new BooleanEvent(loop, this::getC);
    }

    public boolean getJoystick4thFingerTrigger() {
        return getRawButton(Button.Joystick4thFingerTrigger.value);
    }

    public boolean getJoystick4thFingerTriggerPressed() {
        return getRawButtonPressed(Button.Joystick4thFingerTrigger.value);
    }

    public boolean getJoystick4thFingerTriggerReleased() {
        return getRawButtonReleased(Button.Joystick4thFingerTrigger.value);
    }

    public BooleanEvent joystick4thFingerTrigger(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystick4thFingerTrigger);
    }

    public boolean getD() {
        return getRawButton(Button.D.value);
    }

    public boolean getDPressed() {
        return getRawButtonPressed(Button.D.value);
    }

    public boolean getDReleased() {
        return getRawButtonReleased(Button.D.value);
    }

    public BooleanEvent d(EventLoop loop) {
        return new BooleanEvent(loop, this::getD);
    }

    public boolean getE() {
        return getRawButton(Button.E.value);
    }

    public boolean getEPressed() {
        return getRawButtonPressed(Button.E.value);
    }

    public boolean getEReleased() {
        return getRawButtonReleased(Button.E.value);
    }

    public BooleanEvent e(EventLoop loop) {
        return new BooleanEvent(loop, this::getE);
    }
    public boolean getT1Up() {
        return getRawButton(Button.T1Up.value);
    }

    public boolean getT1UpPressed() {
        return getRawButtonPressed(Button.T1Up.value);
    }

    public boolean getT1UpReleased() {
        return getRawButtonReleased(Button.T1Up.value);
    }

    public BooleanEvent t1Up(EventLoop loop) {
        return new BooleanEvent(loop, this::getT1Up);
    }

    public boolean getT1Down() {
        return getRawButton(Button.T1Down.value);
    }

    public boolean getT1DownPressed() {
        return getRawButtonPressed(Button.T1Down.value);
    }

    public boolean getT1DownReleased() {
        return getRawButtonReleased(Button.T1Down.value);
    }

    public BooleanEvent t1Down(EventLoop loop) {
        return new BooleanEvent(loop, this::getT1Down);
    }

    public boolean getT2Up() {
        return getRawButton(Button.T2Up.value);
    }

    public boolean getT2UpPressed() {
        return getRawButtonPressed(Button.T2Up.value);
    }

    public boolean getT2UpReleased() {
        return getRawButtonReleased(Button.T2Up.value);
    }

    public BooleanEvent t2Up(EventLoop loop) {
        return new BooleanEvent(loop, this::getT2Up);
    }

    public boolean getT2Down() {
        return getRawButton(Button.T2Down.value);
    }

    public boolean getT2DownPressed() {
        return getRawButtonPressed(Button.T2Down.value);
    }

    public boolean getT2DownReleased() {
        return getRawButtonReleased(Button.T2Down.value);
    }

    public BooleanEvent t2Down(EventLoop loop) {
        return new BooleanEvent(loop, this::getT2Down);
    }

    public boolean getT3Up() {
        return getRawButton(Button.T3Up.value);
    }

    public boolean getT3UpPressed() {
        return getRawButtonPressed(Button.T3Up.value);
    }

    public boolean getT3UpReleased() {
        return getRawButtonReleased(Button.T3Up.value);
    }

    public BooleanEvent t3Up(EventLoop loop) {
        return new BooleanEvent(loop, this::getT3Up);
    }

    public boolean getT3Down() {
        return getRawButton(Button.T3Down.value);
    }

    public boolean getT3DownPressed() {
        return getRawButtonPressed(Button.T3Down.value);
    }

    public boolean getT3DownReleased() {
        return getRawButtonReleased(Button.T3Down.value);
    }

    public BooleanEvent t3Down(EventLoop loop) {
        return new BooleanEvent(loop, this::getT3Down);
    }

    public boolean getUnknown15() {
        return getRawButton(Button.Unknown15.value);
    }

    public boolean getUnknown15Pressed() {
        return getRawButtonPressed(Button.Unknown15.value);
    }

    public boolean getUnknown15Released() {
        return getRawButtonReleased(Button.Unknown15.value);
    }

    public BooleanEvent unknown15(EventLoop loop) {
        return new BooleanEvent(loop, this::getUnknown15);
    }

    public boolean getMouseButton() {
        return getRawButton(Button.MouseButton.value);
    }

    public boolean getMouseButtonPressed() {
        return getRawButtonPressed(Button.MouseButton.value);
    }

    public boolean getMouseButtonReleased() {
        return getRawButtonReleased(Button.MouseButton.value);
    }

    public BooleanEvent mouseButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getMouseButton);
    }

    public boolean getScrollDownThrottle2ndFinger() {
        return getRawButton(Button.ScrollDownThrottle2ndFinger.value);
    }

    public boolean getScrollDownThrottle2ndFingerPressed() {
        return getRawButtonPressed(Button.ScrollDownThrottle2ndFinger.value);
    }

    public boolean getScrollDownThrottle2ndFingerReleased() {
        return getRawButtonReleased(Button.ScrollDownThrottle2ndFinger.value);
    }

    public BooleanEvent scrollDownThrottle2ndFinger(EventLoop loop) {
        return new BooleanEvent(loop, this::getScrollDownThrottle2ndFinger);
    }

    public boolean getScrollUpThrottle2ndFinger() {
        return getRawButton(Button.ScrollUpThrottle2ndFinger.value);
    }

    public boolean getScrollUpThrottle2ndFingerPressed() {
        return getRawButtonPressed(Button.ScrollUpThrottle2ndFinger.value);
    }

    public boolean getScrollUpThrottle2ndFingerReleased() {
        return getRawButtonReleased(Button.ScrollUpThrottle2ndFinger.value);
    }

    public BooleanEvent scrollUpThrottle2ndFinger(EventLoop loop) {
        return new BooleanEvent(loop, this::getScrollUpThrottle2ndFinger);
    }

    public boolean getScrollPressThrottle2ndFinger() {
        return getRawButton(Button.ScrollPressThrottle2ndFinger.value);
    }

    public boolean getScrollPressThrottle2ndFingerPressed() {
        return getRawButtonPressed(Button.ScrollPressThrottle2ndFinger.value);
    }

    public boolean getScrollPressThrottle2ndFingerReleased() {
        return getRawButtonReleased(Button.ScrollPressThrottle2ndFinger.value);
    }

    public BooleanEvent scrollPressThrottle2ndFinger(EventLoop loop) {
        return new BooleanEvent(loop, this::getScrollPressThrottle2ndFinger);
    }

    public boolean getJoystickBlackPOVUp() {
        return getRawButton(Button.JoystickBlackPOVUp.value);
    }

    public boolean getJoystickBlackPOVUpPressed() {
        return getRawButtonPressed(Button.JoystickBlackPOVUp.value);
    }

    public boolean getJoystickBlackPOVUpReleased() {
        return getRawButtonReleased(Button.JoystickBlackPOVUp.value);
    }

    public BooleanEvent joystickBlackPOVUp(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickBlackPOVUp);
    }

    public boolean getJoystickBlackPOVRight() {
        return getRawButton(Button.JoystickBlackPOVRight.value);
    }

    public boolean getJoystickBlackPOVRightPressed() {
        return getRawButtonPressed(Button.JoystickBlackPOVRight.value);
    }

    public boolean getJoystickBlackPOVRightReleased() {
        return getRawButtonReleased(Button.JoystickBlackPOVRight.value);
    }

    public BooleanEvent joystickBlackPOVRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickBlackPOVRight);
    }

    public boolean getJoystickBlackPOVDown() {
        return getRawButton(Button.JoystickBlackPOVDown.value);
    }

    public boolean getJoystickBlackPOVDownPressed() {
        return getRawButtonPressed(Button.JoystickBlackPOVDown.value);
    }

    public boolean getJoystickBlackPOVDownReleased() {
        return getRawButtonReleased(Button.JoystickBlackPOVDown.value);
    }

    public BooleanEvent joystickBlackPOVDown(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickBlackPOVDown);
    }

    public boolean getJoystickBlackPOVLeft() {
        return getRawButton(Button.JoystickBlackPOVLeft.value);
    }

    public boolean getJoystickBlackPOVLeftPressed() {
        return getRawButtonPressed(Button.JoystickBlackPOVLeft.value);
    }

    public boolean getJoystickBlackPOVLeftReleased() {
        return getRawButtonReleased(Button.JoystickBlackPOVLeft.value);
    }

    public BooleanEvent joystickBlackPOVLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickBlackPOVLeft);
    }

    public boolean getThrottle1stFingerPOVBack() {
        return getRawButton(Button.Throttle1stFingerPOVBack.value);
    }

    public boolean getThrottle1stFingerPOVBackPressed() {
        return getRawButtonPressed(Button.Throttle1stFingerPOVBack.value);
    }

    public boolean getThrottle1stFingerPOVBackReleased() {
        return getRawButtonReleased(Button.Throttle1stFingerPOVBack.value);
    }

    public BooleanEvent throttleFirstFingerPOVBack(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottle1stFingerPOVBack);
    }

    public boolean getThrottle1stFingerPOVLeft() {
        return getRawButton(Button.Throttle1stFingerPOVLeft.value);
    }

    public boolean getThrottle1stFingerPOVLeftPressed() {
        return getRawButtonPressed(Button.Throttle1stFingerPOVLeft.value);
    }

    public boolean getThrottle1stFingerPOVLeftReleased() {
        return getRawButtonReleased(Button.Throttle1stFingerPOVLeft.value);
    }

    public BooleanEvent throttleFirstFingerPOVLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottle1stFingerPOVLeft);
    }

    public boolean getThrottle1stFingerPOVForward() {
        return getRawButton(Button.Throttle1stFingerPOVForward.value);
    }

    public boolean getThrottle1stFingerPOVForwardPressed() {
        return getRawButtonPressed(Button.Throttle1stFingerPOVForward.value);
    }

    public boolean getThrottle1stFingerPOVForwardReleased() {
        return getRawButtonReleased(Button.Throttle1stFingerPOVForward.value);
    }

    public BooleanEvent throttleFirstFingerPOVForward(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottle1stFingerPOVForward);
    }

    public boolean getThrottle1stFingerPOVRight() {
        return getRawButton(Button.Throttle1stFingerPOVRight.value);
    }

    public boolean getThrottle1stFingerPOVRightPressed() {
        return getRawButtonPressed(Button.Throttle1stFingerPOVRight.value);
    }

    public boolean getThrottle1stFingerPOVRightReleased() {
        return getRawButtonReleased(Button.Throttle1stFingerPOVRight.value);
    }

    public BooleanEvent throttleFirstFingerPOVRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottle1stFingerPOVRight);
    }

    public boolean getUnknown28() {
        return getRawButton(Button.Unknown28.value);
    }

    public boolean getUnknown28Pressed() {
        return getRawButtonPressed(Button.Unknown28.value);
    }

    public boolean getUnknown28Released() {
        return getRawButtonReleased(Button.Unknown28.value);
    }

    public BooleanEvent unknown28(EventLoop loop) {
        return new BooleanEvent(loop, this::getUnknown28);
    }

    public boolean getUnknown29() {
        return getRawButton(Button.Unknown29.value);
    }

    public boolean getUnknown29Pressed() {
        return getRawButtonPressed(Button.Unknown29.value);
    }

    public boolean getUnknown29Released() {
        return getRawButtonReleased(Button.Unknown29.value);
    }

    public BooleanEvent unknown29(EventLoop loop) {
        return new BooleanEvent(loop, this::getUnknown29);
    }

    public boolean getUnknown30() {
        return getRawButton(Button.Unknown30.value);
    }

    public boolean getUnknown30Pressed() {
        return getRawButtonPressed(Button.Unknown30.value);
    }

    public boolean getUnknown30Released() {
        return getRawButtonReleased(Button.Unknown30.value);
    }

    public BooleanEvent unknown30(EventLoop loop) {
        return new BooleanEvent(loop, this::getUnknown30);
    }

    public boolean getUnknown31() {
        return getRawButton(Button.Unknown31.value);
    }

    public boolean getUnknown31Pressed() {
        return getRawButtonPressed(Button.Unknown31.value);
    }

    public boolean getUnknown31Released() {
        return getRawButtonReleased(Button.Unknown31.value);
    }

    public BooleanEvent unknown31(EventLoop loop) {
        return new BooleanEvent(loop, this::getUnknown31);
    }

    public boolean getUnknown32() {
        return getRawButton(Button.Unknown32.value);
    }

    public boolean getUnknown32Pressed() {
        return getRawButtonPressed(Button.Unknown32.value);
    }

    public boolean getUnknown32Released() {
        return getRawButtonReleased(Button.Unknown32.value);
    }

    public BooleanEvent unknown32(EventLoop loop) {
        return new BooleanEvent(loop, this::getUnknown32);
    }
}
