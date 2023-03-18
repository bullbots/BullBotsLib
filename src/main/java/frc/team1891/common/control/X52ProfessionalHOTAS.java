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
        JoystickX(0),
        JoystickY(1),
        Throttle(2),
        ThrottleThumbDial(3),
        Throttle1stFingerDial(4),
        JoystickZ(5),
        ThrottleThumbSlider(6);

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
        T1(9),
        T2(10),
        T3(11),
        T4(12),
        T5(13),
        T6(14),
        JoystickTriggerSecondLevel(15),
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
        JoystickModeRed(28),
        JoystickModePurple(29),
        JoystickModeBlue(30),
        ThrottleThumbDialButton(31),
        LeftScrollPressMultiFunctionDisplay(32);
        // TODO: There may be more buttons that don't show up in the sim

        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    public enum Mode {
        RED(28),
        PURPLE(29),
        BLUE(30);

        public final int buttonNumber;

        Mode(int buttonNumber) {
            this.buttonNumber = buttonNumber;
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

    public double getThrottleThumbDial() {
        return getRawAxis(Axis.ThrottleThumbDial.value);
    }

    public double getThrottle1stFingerDial() {
        return getRawAxis(Axis.Throttle1stFingerDial.value);
    }

    public double getJoystickZ() {
        return getRawAxis(Axis.JoystickZ.value);
    }

    public double getThrottleThumbSlider() {
        return getRawAxis(Axis.ThrottleThumbSlider.value);
    }

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
    public boolean getT1() {
        return getRawButton(Button.T1.value);
    }

    public boolean getT1Pressed() {
        return getRawButtonPressed(Button.T1.value);
    }

    public boolean getT1Released() {
        return getRawButtonReleased(Button.T1.value);
    }

    public BooleanEvent t1(EventLoop loop) {
        return new BooleanEvent(loop, this::getT1);
    }

    public boolean getT2() {
        return getRawButton(Button.T2.value);
    }

    public boolean getT2Pressed() {
        return getRawButtonPressed(Button.T2.value);
    }

    public boolean getT2Released() {
        return getRawButtonReleased(Button.T2.value);
    }

    public BooleanEvent t2(EventLoop loop) {
        return new BooleanEvent(loop, this::getT2);
    }

    public boolean getT3() {
        return getRawButton(Button.T3.value);
    }

    public boolean getT3Pressed() {
        return getRawButtonPressed(Button.T3.value);
    }

    public boolean getT3Released() {
        return getRawButtonReleased(Button.T3.value);
    }

    public BooleanEvent t3(EventLoop loop) {
        return new BooleanEvent(loop, this::getT3);
    }

    public boolean getT4() {
        return getRawButton(Button.T4.value);
    }

    public boolean getT4Pressed() {
        return getRawButtonPressed(Button.T4.value);
    }

    public boolean getT4Released() {
        return getRawButtonReleased(Button.T4.value);
    }

    public BooleanEvent t4(EventLoop loop) {
        return new BooleanEvent(loop, this::getT4);
    }

    public boolean getT5() {
        return getRawButton(Button.T5.value);
    }

    public boolean getT5Pressed() {
        return getRawButtonPressed(Button.T5.value);
    }

    public boolean getT5Released() {
        return getRawButtonReleased(Button.T5.value);
    }

    public BooleanEvent t5(EventLoop loop) {
        return new BooleanEvent(loop, this::getT5);
    }

    public boolean getT6() {
        return getRawButton(Button.T6.value);
    }

    public boolean getT6Pressed() {
        return getRawButtonPressed(Button.T6.value);
    }

    public boolean getT6Released() {
        return getRawButtonReleased(Button.T6.value);
    }

    public BooleanEvent t6(EventLoop loop) {
        return new BooleanEvent(loop, this::getT6);
    }

    public boolean getJoystickTriggerSecondLevel() {
        return getRawButton(Button.JoystickTriggerSecondLevel.value);
    }

    public boolean getJoystickTriggerSecondLevelPressed() {
        return getRawButtonPressed(Button.JoystickTriggerSecondLevel.value);
    }

    public boolean getJoystickTriggerSecondLevelReleased() {
        return getRawButtonReleased(Button.JoystickTriggerSecondLevel.value);
    }

    public BooleanEvent joystickTriggerSecondLevel(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickTriggerSecondLevel);
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

    public boolean getJoystickModeRed() {
        return getRawButton(Button.JoystickModeRed.value);
    }

    public boolean getJoystickModeRedPressed() {
        return getRawButtonPressed(Button.JoystickModeRed.value);
    }

    public boolean getJoystickModeRedReleased() {
        return getRawButtonReleased(Button.JoystickModeRed.value);
    }

    public BooleanEvent joystickModeRed(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickModeRed);
    }

    public boolean getJoystickModePurple() {
        return getRawButton(Button.JoystickModePurple.value);
    }

    public boolean getJoystickModePurplePressed() {
        return getRawButtonPressed(Button.JoystickModePurple.value);
    }

    public boolean getJoystickModePurpleReleased() {
        return getRawButtonReleased(Button.JoystickModePurple.value);
    }

    public BooleanEvent joystickModePurple(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickModePurple);
    }

    public boolean getJoystickModeBlue() {
        return getRawButton(Button.JoystickModeBlue.value);
    }

    public boolean getJoystickModeBluePressed() {
        return getRawButtonPressed(Button.JoystickModeBlue.value);
    }

    public boolean getJoystickModeBlueReleased() {
        return getRawButtonReleased(Button.JoystickModeBlue.value);
    }

    public BooleanEvent joystickModeBlue(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickModeBlue);
    }

    public boolean getThrottleThumbDialButton() {
        return getRawButton(Button.ThrottleThumbDialButton.value);
    }

    public boolean getThrottleThumbDialButtonPressed() {
        return getRawButtonPressed(Button.ThrottleThumbDialButton.value);
    }

    public boolean getThrottleThumbDialButtonReleased() {
        return getRawButtonReleased(Button.ThrottleThumbDialButton.value);
    }

    public BooleanEvent throttleThumbDialButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottleThumbDialButton);
    }

    public boolean getLeftScrollPressMultiFunctionDisplay() {
        return getRawButton(Button.LeftScrollPressMultiFunctionDisplay.value);
    }

    public boolean getLeftScrollPressMultiFunctionDisplayPressed() {
        return getRawButtonPressed(Button.LeftScrollPressMultiFunctionDisplay.value);
    }

    public boolean getLeftScrollPressMultiFunctionDisplayReleased() {
        return getRawButtonReleased(Button.LeftScrollPressMultiFunctionDisplay.value);
    }

    public BooleanEvent leftScrollPressMultiFunctionalDisplayRelease(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftScrollPressMultiFunctionDisplay);
    }

    public Mode getMode() {
        if (getJoystickModeRed()) {
            return Mode.RED;
        }
        if (getJoystickModePurple()) {
            return Mode.PURPLE;
        }
        if (getJoystickModeBlue()) {
            return Mode.BLUE;
        }
        return null;
    }
}
