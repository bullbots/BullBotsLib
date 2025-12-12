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
    /** Axis mappings for the X52 Professional HOTAS controller. */
    public enum Axis {
        /** Joystick X axis (left/right movement). */
        JoystickX(0),
        /** Joystick Y axis (forward/backward movement). */
        JoystickY(1),
        /** Throttle axis. */
        Throttle(2),
        /** Throttle thumb dial axis. */
        ThrottleThumbDial(3),
        /** Throttle 1st finger dial axis. */
        Throttle1stFingerDial(4),
        /** Joystick Z axis (twist rotation). */
        JoystickZ(5),
        /** Throttle thumb slider axis. */
        ThrottleThumbSlider(6);

        /** The axis ID value. */
        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    /** Button mappings for the X52 Professional HOTAS controller. */
    public enum Button {
        /** Joystick trigger first level. */
        JoystickTriggerFirstLevel(1),
        /** Fire button. */
        Fire(2),
        /** A button. */
        A(3),
        /** B button. */
        B(4),
        /** C button. */
        C(5),
        /** Joystick 4th finger trigger. */
        Joystick4thFingerTrigger(6),
        /** D button. */
        D(7),
        /** E button. */
        E(8),
        /** T1 button on throttle base. */
        T1(9),
        /** T2 button on throttle base. */
        T2(10),
        /** T3 button on throttle base. */
        T3(11),
        /** T4 button on throttle base. */
        T4(12),
        /** T5 button on throttle base. */
        T5(13),
        /** T6 button on throttle base. */
        T6(14),
        /** Joystick trigger second level. */
        JoystickTriggerSecondLevel(15),
        /** Mouse button on throttle. */
        MouseButton(16),
        /** Scroll down button on throttle 2nd finger. */
        ScrollDownThrottle2ndFinger(17),
        /** Scroll up button on throttle 2nd finger. */
        ScrollUpThrottle2ndFinger(18),
        /** Scroll press button on throttle 2nd finger. */
        ScrollPressThrottle2ndFinger(19),
        /** Joystick black POV up. */
        JoystickBlackPOVUp(20),
        /** Joystick black POV right. */
        JoystickBlackPOVRight(21),
        /** Joystick black POV down. */
        JoystickBlackPOVDown(22),
        /** Joystick black POV left. */
        JoystickBlackPOVLeft(23),
        /** Throttle 1st finger POV back. */
        Throttle1stFingerPOVBack(24),
        /** Throttle 1st finger POV left. */
        Throttle1stFingerPOVLeft(25),
        /** Throttle 1st finger POV forward. */
        Throttle1stFingerPOVForward(26),
        /** Throttle 1st finger POV right. */
        Throttle1stFingerPOVRight(27),
        /** Joystick mode red. */
        JoystickModeRed(28),
        /** Joystick mode purple. */
        JoystickModePurple(29),
        /** Joystick mode blue. */
        JoystickModeBlue(30),
        /** Throttle thumb dial button. */
        ThrottleThumbDialButton(31),
        /** Left scroll press on multi-function display. */
        LeftScrollPressMultiFunctionDisplay(32);
        // TODO: There may be more buttons that don't show up in the sim

        /** The button ID value. */
        public final int value;

        Button(int value) {
            this.value = value;
        }
    }

    /** Mode selector positions for the X52 Professional HOTAS controller. */
    public enum Mode {
        /** Red mode position. */
        RED(28),
        /** Purple mode position. */
        PURPLE(29),
        /** Blue mode position. */
        BLUE(30);

        /** The button number associated with this mode. */
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

    /** @return the joystick X axis value */
    public double getJoystickX() {
        return getRawAxis(Axis.JoystickX.value);
    }

    /** @return the joystick Y axis value */
    public double getJoystickY() {
        return getRawAxis(Axis.JoystickY.value);
    }

    /** @return the throttle axis value */
    public double getThrottle() {
        return getRawAxis(Axis.Throttle.value);
    }

    /** @return the throttle thumb dial axis value */
    public double getThrottleThumbDial() {
        return getRawAxis(Axis.ThrottleThumbDial.value);
    }

    /** @return the throttle 1st finger dial axis value */
    public double getThrottle1stFingerDial() {
        return getRawAxis(Axis.Throttle1stFingerDial.value);
    }

    /** @return the joystick Z (twist) axis value */
    public double getJoystickZ() {
        return getRawAxis(Axis.JoystickZ.value);
    }

    /** @return the throttle thumb slider axis value */
    public double getThrottleThumbSlider() {
        return getRawAxis(Axis.ThrottleThumbSlider.value);
    }

    /** @return whether the joystick trigger first level is currently pressed */
    public boolean getJoystickTriggerFirstLevel() {
        return getRawButton(Button.JoystickTriggerFirstLevel.value);
    }

    /** @return whether the joystick trigger first level was pressed this frame */
    public boolean getJoystickTriggerFirstLevelPressed() {
        return getRawButtonPressed(Button.JoystickTriggerFirstLevel.value);
    }

    /** @return whether the joystick trigger first level was released this frame */
    public boolean getJoystickTriggerFirstLevelReleased() {
        return getRawButtonReleased(Button.JoystickTriggerFirstLevel.value);
    }

    /**
     * Creates a BooleanEvent for the joystick trigger first level.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickTriggerfirstLevel(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickTriggerFirstLevel);
    }

    /** @return whether the fire button is currently pressed */
    public boolean getFire() {
        return getRawButton(Button.Fire.value);
    }

    /** @return whether the fire button was pressed this frame */
    public boolean getFirePressed() {
        return getRawButtonPressed(Button.Fire.value);
    }

    /** @return whether the fire button was released this frame */
    public boolean getFireReleased() {
        return getRawButtonReleased(Button.Fire.value);
    }

    /**
     * Creates a BooleanEvent for the fire button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent fire(EventLoop loop) {
        return new BooleanEvent(loop, this::getFire);
    }

    /** @return whether the A button is currently pressed */
    public boolean getA() {
        return getRawButton(Button.A.value);
    }

    /** @return whether the A button was pressed this frame */
    public boolean getAPressed() {
        return getRawButtonPressed(Button.A.value);
    }

    /** @return whether the A button was released this frame */
    public boolean getAReleased() {
        return getRawButtonReleased(Button.A.value);
    }

    /**
     * Creates a BooleanEvent for the A button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent a(EventLoop loop) {
        return new BooleanEvent(loop, this::getA);
    }

    /** @return whether the B button is currently pressed */
    public boolean getB() {
        return getRawButton(Button.B.value);
    }

    /** @return whether the B button was pressed this frame */
    public boolean getBPressed() {
        return getRawButtonPressed(Button.B.value);
    }

    /** @return whether the B button was released this frame */
    public boolean getBReleased() {
        return getRawButtonReleased(Button.B.value);
    }

    /**
     * Creates a BooleanEvent for the B button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent b(EventLoop loop) {
        return new BooleanEvent(loop, this::getB);
    }

    /** @return whether the C button is currently pressed */
    public boolean getC() {
        return getRawButton(Button.C.value);
    }

    /** @return whether the C button was pressed this frame */
    public boolean getCPressed() {
        return getRawButtonPressed(Button.C.value);
    }

    /** @return whether the C button was released this frame */
    public boolean getCReleased() {
        return getRawButtonReleased(Button.C.value);
    }

    /**
     * Creates a BooleanEvent for the C button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent c(EventLoop loop) {
        return new BooleanEvent(loop, this::getC);
    }

    /** @return whether the joystick 4th finger trigger is currently pressed */
    public boolean getJoystick4thFingerTrigger() {
        return getRawButton(Button.Joystick4thFingerTrigger.value);
    }

    /** @return whether the joystick 4th finger trigger was pressed this frame */
    public boolean getJoystick4thFingerTriggerPressed() {
        return getRawButtonPressed(Button.Joystick4thFingerTrigger.value);
    }

    /** @return whether the joystick 4th finger trigger was released this frame */
    public boolean getJoystick4thFingerTriggerReleased() {
        return getRawButtonReleased(Button.Joystick4thFingerTrigger.value);
    }

    /**
     * Creates a BooleanEvent for the joystick 4th finger trigger.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystick4thFingerTrigger(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystick4thFingerTrigger);
    }

    /** @return whether the D button is currently pressed */
    public boolean getD() {
        return getRawButton(Button.D.value);
    }

    /** @return whether the D button was pressed this frame */
    public boolean getDPressed() {
        return getRawButtonPressed(Button.D.value);
    }

    /** @return whether the D button was released this frame */
    public boolean getDReleased() {
        return getRawButtonReleased(Button.D.value);
    }

    /**
     * Creates a BooleanEvent for the D button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent d(EventLoop loop) {
        return new BooleanEvent(loop, this::getD);
    }

    /** @return whether the E button is currently pressed */
    public boolean getE() {
        return getRawButton(Button.E.value);
    }

    /** @return whether the E button was pressed this frame */
    public boolean getEPressed() {
        return getRawButtonPressed(Button.E.value);
    }

    /** @return whether the E button was released this frame */
    public boolean getEReleased() {
        return getRawButtonReleased(Button.E.value);
    }

    /**
     * Creates a BooleanEvent for the E button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent e(EventLoop loop) {
        return new BooleanEvent(loop, this::getE);
    }

    /** @return whether the T1 button is currently pressed */
    public boolean getT1() {
        return getRawButton(Button.T1.value);
    }

    /** @return whether the T1 button was pressed this frame */
    public boolean getT1Pressed() {
        return getRawButtonPressed(Button.T1.value);
    }

    /** @return whether the T1 button was released this frame */
    public boolean getT1Released() {
        return getRawButtonReleased(Button.T1.value);
    }

    /**
     * Creates a BooleanEvent for the T1 button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent t1(EventLoop loop) {
        return new BooleanEvent(loop, this::getT1);
    }

    /** @return whether the T2 button is currently pressed */
    public boolean getT2() {
        return getRawButton(Button.T2.value);
    }

    /** @return whether the T2 button was pressed this frame */
    public boolean getT2Pressed() {
        return getRawButtonPressed(Button.T2.value);
    }

    /** @return whether the T2 button was released this frame */
    public boolean getT2Released() {
        return getRawButtonReleased(Button.T2.value);
    }

    /**
     * Creates a BooleanEvent for the T2 button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent t2(EventLoop loop) {
        return new BooleanEvent(loop, this::getT2);
    }

    /** @return whether the T3 button is currently pressed */
    public boolean getT3() {
        return getRawButton(Button.T3.value);
    }

    /** @return whether the T3 button was pressed this frame */
    public boolean getT3Pressed() {
        return getRawButtonPressed(Button.T3.value);
    }

    /** @return whether the T3 button was released this frame */
    public boolean getT3Released() {
        return getRawButtonReleased(Button.T3.value);
    }

    /**
     * Creates a BooleanEvent for the T3 button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent t3(EventLoop loop) {
        return new BooleanEvent(loop, this::getT3);
    }

    /** @return whether the T4 button is currently pressed */
    public boolean getT4() {
        return getRawButton(Button.T4.value);
    }

    /** @return whether the T4 button was pressed this frame */
    public boolean getT4Pressed() {
        return getRawButtonPressed(Button.T4.value);
    }

    /** @return whether the T4 button was released this frame */
    public boolean getT4Released() {
        return getRawButtonReleased(Button.T4.value);
    }

    /**
     * Creates a BooleanEvent for the T4 button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent t4(EventLoop loop) {
        return new BooleanEvent(loop, this::getT4);
    }

    /** @return whether the T5 button is currently pressed */
    public boolean getT5() {
        return getRawButton(Button.T5.value);
    }

    /** @return whether the T5 button was pressed this frame */
    public boolean getT5Pressed() {
        return getRawButtonPressed(Button.T5.value);
    }

    /** @return whether the T5 button was released this frame */
    public boolean getT5Released() {
        return getRawButtonReleased(Button.T5.value);
    }

    /**
     * Creates a BooleanEvent for the T5 button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent t5(EventLoop loop) {
        return new BooleanEvent(loop, this::getT5);
    }

    /** @return whether the T6 button is currently pressed */
    public boolean getT6() {
        return getRawButton(Button.T6.value);
    }

    /** @return whether the T6 button was pressed this frame */
    public boolean getT6Pressed() {
        return getRawButtonPressed(Button.T6.value);
    }

    /** @return whether the T6 button was released this frame */
    public boolean getT6Released() {
        return getRawButtonReleased(Button.T6.value);
    }

    /**
     * Creates a BooleanEvent for the T6 button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent t6(EventLoop loop) {
        return new BooleanEvent(loop, this::getT6);
    }

    /** @return whether the joystick trigger second level is currently pressed */
    public boolean getJoystickTriggerSecondLevel() {
        return getRawButton(Button.JoystickTriggerSecondLevel.value);
    }

    /** @return whether the joystick trigger second level was pressed this frame */
    public boolean getJoystickTriggerSecondLevelPressed() {
        return getRawButtonPressed(Button.JoystickTriggerSecondLevel.value);
    }

    /** @return whether the joystick trigger second level was released this frame */
    public boolean getJoystickTriggerSecondLevelReleased() {
        return getRawButtonReleased(Button.JoystickTriggerSecondLevel.value);
    }

    /**
     * Creates a BooleanEvent for the joystick trigger second level.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickTriggerSecondLevel(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickTriggerSecondLevel);
    }

    /** @return whether the mouse button is currently pressed */
    public boolean getMouseButton() {
        return getRawButton(Button.MouseButton.value);
    }

    /** @return whether the mouse button was pressed this frame */
    public boolean getMouseButtonPressed() {
        return getRawButtonPressed(Button.MouseButton.value);
    }

    /** @return whether the mouse button was released this frame */
    public boolean getMouseButtonReleased() {
        return getRawButtonReleased(Button.MouseButton.value);
    }

    /**
     * Creates a BooleanEvent for the mouse button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent mouseButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getMouseButton);
    }

    /** @return whether the scroll down throttle 2nd finger button is currently pressed */
    public boolean getScrollDownThrottle2ndFinger() {
        return getRawButton(Button.ScrollDownThrottle2ndFinger.value);
    }

    /** @return whether the scroll down throttle 2nd finger button was pressed this frame */
    public boolean getScrollDownThrottle2ndFingerPressed() {
        return getRawButtonPressed(Button.ScrollDownThrottle2ndFinger.value);
    }

    /** @return whether the scroll down throttle 2nd finger button was released this frame */
    public boolean getScrollDownThrottle2ndFingerReleased() {
        return getRawButtonReleased(Button.ScrollDownThrottle2ndFinger.value);
    }

    /**
     * Creates a BooleanEvent for the scroll down throttle 2nd finger button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent scrollDownThrottle2ndFinger(EventLoop loop) {
        return new BooleanEvent(loop, this::getScrollDownThrottle2ndFinger);
    }

    /** @return whether the scroll up throttle 2nd finger button is currently pressed */
    public boolean getScrollUpThrottle2ndFinger() {
        return getRawButton(Button.ScrollUpThrottle2ndFinger.value);
    }

    /** @return whether the scroll up throttle 2nd finger button was pressed this frame */
    public boolean getScrollUpThrottle2ndFingerPressed() {
        return getRawButtonPressed(Button.ScrollUpThrottle2ndFinger.value);
    }

    /** @return whether the scroll up throttle 2nd finger button was released this frame */
    public boolean getScrollUpThrottle2ndFingerReleased() {
        return getRawButtonReleased(Button.ScrollUpThrottle2ndFinger.value);
    }

    /**
     * Creates a BooleanEvent for the scroll up throttle 2nd finger button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent scrollUpThrottle2ndFinger(EventLoop loop) {
        return new BooleanEvent(loop, this::getScrollUpThrottle2ndFinger);
    }

    /** @return whether the scroll press throttle 2nd finger button is currently pressed */
    public boolean getScrollPressThrottle2ndFinger() {
        return getRawButton(Button.ScrollPressThrottle2ndFinger.value);
    }

    /** @return whether the scroll press throttle 2nd finger button was pressed this frame */
    public boolean getScrollPressThrottle2ndFingerPressed() {
        return getRawButtonPressed(Button.ScrollPressThrottle2ndFinger.value);
    }

    /** @return whether the scroll press throttle 2nd finger button was released this frame */
    public boolean getScrollPressThrottle2ndFingerReleased() {
        return getRawButtonReleased(Button.ScrollPressThrottle2ndFinger.value);
    }

    /**
     * Creates a BooleanEvent for the scroll press throttle 2nd finger button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent scrollPressThrottle2ndFinger(EventLoop loop) {
        return new BooleanEvent(loop, this::getScrollPressThrottle2ndFinger);
    }

    /** @return whether the joystick black POV up button is currently pressed */
    public boolean getJoystickBlackPOVUp() {
        return getRawButton(Button.JoystickBlackPOVUp.value);
    }

    /** @return whether the joystick black POV up button was pressed this frame */
    public boolean getJoystickBlackPOVUpPressed() {
        return getRawButtonPressed(Button.JoystickBlackPOVUp.value);
    }

    /** @return whether the joystick black POV up button was released this frame */
    public boolean getJoystickBlackPOVUpReleased() {
        return getRawButtonReleased(Button.JoystickBlackPOVUp.value);
    }

    /**
     * Creates a BooleanEvent for the joystick black POV up button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickBlackPOVUp(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickBlackPOVUp);
    }

    /** @return whether the joystick black POV right button is currently pressed */
    public boolean getJoystickBlackPOVRight() {
        return getRawButton(Button.JoystickBlackPOVRight.value);
    }

    /** @return whether the joystick black POV right button was pressed this frame */
    public boolean getJoystickBlackPOVRightPressed() {
        return getRawButtonPressed(Button.JoystickBlackPOVRight.value);
    }

    /** @return whether the joystick black POV right button was released this frame */
    public boolean getJoystickBlackPOVRightReleased() {
        return getRawButtonReleased(Button.JoystickBlackPOVRight.value);
    }

    /**
     * Creates a BooleanEvent for the joystick black POV right button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickBlackPOVRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickBlackPOVRight);
    }

    /** @return whether the joystick black POV down button is currently pressed */
    public boolean getJoystickBlackPOVDown() {
        return getRawButton(Button.JoystickBlackPOVDown.value);
    }

    /** @return whether the joystick black POV down button was pressed this frame */
    public boolean getJoystickBlackPOVDownPressed() {
        return getRawButtonPressed(Button.JoystickBlackPOVDown.value);
    }

    /** @return whether the joystick black POV down button was released this frame */
    public boolean getJoystickBlackPOVDownReleased() {
        return getRawButtonReleased(Button.JoystickBlackPOVDown.value);
    }

    /**
     * Creates a BooleanEvent for the joystick black POV down button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickBlackPOVDown(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickBlackPOVDown);
    }

    /** @return whether the joystick black POV left button is currently pressed */
    public boolean getJoystickBlackPOVLeft() {
        return getRawButton(Button.JoystickBlackPOVLeft.value);
    }

    /** @return whether the joystick black POV left button was pressed this frame */
    public boolean getJoystickBlackPOVLeftPressed() {
        return getRawButtonPressed(Button.JoystickBlackPOVLeft.value);
    }

    /** @return whether the joystick black POV left button was released this frame */
    public boolean getJoystickBlackPOVLeftReleased() {
        return getRawButtonReleased(Button.JoystickBlackPOVLeft.value);
    }

    /**
     * Creates a BooleanEvent for the joystick black POV left button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickBlackPOVLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickBlackPOVLeft);
    }

    /** @return whether the throttle 1st finger POV back button is currently pressed */
    public boolean getThrottle1stFingerPOVBack() {
        return getRawButton(Button.Throttle1stFingerPOVBack.value);
    }

    /** @return whether the throttle 1st finger POV back button was pressed this frame */
    public boolean getThrottle1stFingerPOVBackPressed() {
        return getRawButtonPressed(Button.Throttle1stFingerPOVBack.value);
    }

    /** @return whether the throttle 1st finger POV back button was released this frame */
    public boolean getThrottle1stFingerPOVBackReleased() {
        return getRawButtonReleased(Button.Throttle1stFingerPOVBack.value);
    }

    /**
     * Creates a BooleanEvent for the throttle 1st finger POV back button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent throttleFirstFingerPOVBack(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottle1stFingerPOVBack);
    }

    /** @return whether the throttle 1st finger POV left button is currently pressed */
    public boolean getThrottle1stFingerPOVLeft() {
        return getRawButton(Button.Throttle1stFingerPOVLeft.value);
    }

    /** @return whether the throttle 1st finger POV left button was pressed this frame */
    public boolean getThrottle1stFingerPOVLeftPressed() {
        return getRawButtonPressed(Button.Throttle1stFingerPOVLeft.value);
    }

    /** @return whether the throttle 1st finger POV left button was released this frame */
    public boolean getThrottle1stFingerPOVLeftReleased() {
        return getRawButtonReleased(Button.Throttle1stFingerPOVLeft.value);
    }

    /**
     * Creates a BooleanEvent for the throttle 1st finger POV left button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent throttleFirstFingerPOVLeft(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottle1stFingerPOVLeft);
    }

    /** @return whether the throttle 1st finger POV forward button is currently pressed */
    public boolean getThrottle1stFingerPOVForward() {
        return getRawButton(Button.Throttle1stFingerPOVForward.value);
    }

    /** @return whether the throttle 1st finger POV forward button was pressed this frame */
    public boolean getThrottle1stFingerPOVForwardPressed() {
        return getRawButtonPressed(Button.Throttle1stFingerPOVForward.value);
    }

    /** @return whether the throttle 1st finger POV forward button was released this frame */
    public boolean getThrottle1stFingerPOVForwardReleased() {
        return getRawButtonReleased(Button.Throttle1stFingerPOVForward.value);
    }

    /**
     * Creates a BooleanEvent for the throttle 1st finger POV forward button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent throttleFirstFingerPOVForward(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottle1stFingerPOVForward);
    }

    /** @return whether the throttle 1st finger POV right button is currently pressed */
    public boolean getThrottle1stFingerPOVRight() {
        return getRawButton(Button.Throttle1stFingerPOVRight.value);
    }

    /** @return whether the throttle 1st finger POV right button was pressed this frame */
    public boolean getThrottle1stFingerPOVRightPressed() {
        return getRawButtonPressed(Button.Throttle1stFingerPOVRight.value);
    }

    /** @return whether the throttle 1st finger POV right button was released this frame */
    public boolean getThrottle1stFingerPOVRightReleased() {
        return getRawButtonReleased(Button.Throttle1stFingerPOVRight.value);
    }

    /**
     * Creates a BooleanEvent for the throttle 1st finger POV right button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent throttleFirstFingerPOVRight(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottle1stFingerPOVRight);
    }

    /** @return whether the joystick mode red button is currently pressed */
    public boolean getJoystickModeRed() {
        return getRawButton(Button.JoystickModeRed.value);
    }

    /** @return whether the joystick mode red button was pressed this frame */
    public boolean getJoystickModeRedPressed() {
        return getRawButtonPressed(Button.JoystickModeRed.value);
    }

    /** @return whether the joystick mode red button was released this frame */
    public boolean getJoystickModeRedReleased() {
        return getRawButtonReleased(Button.JoystickModeRed.value);
    }

    /**
     * Creates a BooleanEvent for the joystick mode red button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickModeRed(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickModeRed);
    }

    /** @return whether the joystick mode purple button is currently pressed */
    public boolean getJoystickModePurple() {
        return getRawButton(Button.JoystickModePurple.value);
    }

    /** @return whether the joystick mode purple button was pressed this frame */
    public boolean getJoystickModePurplePressed() {
        return getRawButtonPressed(Button.JoystickModePurple.value);
    }

    /** @return whether the joystick mode purple button was released this frame */
    public boolean getJoystickModePurpleReleased() {
        return getRawButtonReleased(Button.JoystickModePurple.value);
    }

    /**
     * Creates a BooleanEvent for the joystick mode purple button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickModePurple(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickModePurple);
    }

    /** @return whether the joystick mode blue button is currently pressed */
    public boolean getJoystickModeBlue() {
        return getRawButton(Button.JoystickModeBlue.value);
    }

    /** @return whether the joystick mode blue button was pressed this frame */
    public boolean getJoystickModeBluePressed() {
        return getRawButtonPressed(Button.JoystickModeBlue.value);
    }

    /** @return whether the joystick mode blue button was released this frame */
    public boolean getJoystickModeBlueReleased() {
        return getRawButtonReleased(Button.JoystickModeBlue.value);
    }

    /**
     * Creates a BooleanEvent for the joystick mode blue button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent joystickModeBlue(EventLoop loop) {
        return new BooleanEvent(loop, this::getJoystickModeBlue);
    }

    /** @return whether the throttle thumb dial button is currently pressed */
    public boolean getThrottleThumbDialButton() {
        return getRawButton(Button.ThrottleThumbDialButton.value);
    }

    /** @return whether the throttle thumb dial button was pressed this frame */
    public boolean getThrottleThumbDialButtonPressed() {
        return getRawButtonPressed(Button.ThrottleThumbDialButton.value);
    }

    /** @return whether the throttle thumb dial button was released this frame */
    public boolean getThrottleThumbDialButtonReleased() {
        return getRawButtonReleased(Button.ThrottleThumbDialButton.value);
    }

    /**
     * Creates a BooleanEvent for the throttle thumb dial button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent throttleThumbDialButton(EventLoop loop) {
        return new BooleanEvent(loop, this::getThrottleThumbDialButton);
    }

    /** @return whether the left scroll press multi-function display button is currently pressed */
    public boolean getLeftScrollPressMultiFunctionDisplay() {
        return getRawButton(Button.LeftScrollPressMultiFunctionDisplay.value);
    }

    /** @return whether the left scroll press multi-function display button was pressed this frame */
    public boolean getLeftScrollPressMultiFunctionDisplayPressed() {
        return getRawButtonPressed(Button.LeftScrollPressMultiFunctionDisplay.value);
    }

    /** @return whether the left scroll press multi-function display button was released this frame */
    public boolean getLeftScrollPressMultiFunctionDisplayReleased() {
        return getRawButtonReleased(Button.LeftScrollPressMultiFunctionDisplay.value);
    }

    /**
     * Creates a BooleanEvent for the left scroll press multi-function display button.
     * @param loop the event loop
     * @return the boolean event
     */
    public BooleanEvent leftScrollPressMultiFunctionalDisplayRelease(EventLoop loop) {
        return new BooleanEvent(loop, this::getLeftScrollPressMultiFunctionDisplay);
    }

    /** @return the currently selected mode or null if no mode is active */
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
