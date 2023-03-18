// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;

/** A controller that detects the device plugged in, and uses the correct inputs to control the robot. */
@SuppressWarnings("unused")
public class SmartController extends GenericHID {
    protected static final boolean[] default_axis_inverted = new boolean[] {
        false,
        false,
        false,
        false
    };

    public enum AxisType {
        Strafe(0),
        Forward(1),
        Twist(2),
        Throttle(3);

        public final int value;

        AxisType(int value) {
            this.value = value;
        }

        @Override
        public String toString() {
            String name = this.name().substring(1);
            return name + "Axis";
        }
    }
    private final byte[] axes = new byte[AxisType.values().length];
    private final boolean[] axis_inverted = new boolean[AxisType.values().length];

    private final HashMap<String, Trigger> triggers = new HashMap<>();

    private boolean hasThrottle;

    public SmartController(int port) {
        super(port);

        System.out.printf("""
                Info: SmartController(%d)
                \tName: "%s"
                \tType: HIDType.%s
                """, port, getName(), getType());
        configure();
    }

    public void configure() {
        System.out.printf("Info: SmartController(%d) ", getPort());

        axis_inverted[AxisType.Strafe.value] = default_axis_inverted[AxisType.Strafe.value];
        axis_inverted[AxisType.Forward.value] = default_axis_inverted[AxisType.Forward.value];
        axis_inverted[AxisType.Twist.value] = default_axis_inverted[AxisType.Twist.value];
        axis_inverted[AxisType.Throttle.value] = default_axis_inverted[AxisType.Throttle.value];

        if (getName().equals("Logitech Extreme 3D")) {
            System.out.println("Logitech Extreme 3D detected!");
            axes[AxisType.Strafe.value] = 0;
            axes[AxisType.Forward.value] = 1;
            axes[AxisType.Twist.value] = 2;
            axes[AxisType.Throttle.value] = 3;

            hasThrottle = true;
        } else if (getName().equals("Logitech Attack 3")) {
            System.out.println("Logitech Attack 3 detected!");
            axes[AxisType.Strafe.value] = -1;
            axes[AxisType.Forward.value] = 1;
            axes[AxisType.Twist.value] = 0;
            axes[AxisType.Throttle.value] = 3;

            hasThrottle = true;
        } else if (getName().contains("Logitech Dual Action")) {
            System.out.println("Logitech Dual Action detected!");
            axes[AxisType.Strafe.value] = (byte) XboxController.Axis.kLeftX.value;
            axes[AxisType.Forward.value] = (byte) XboxController.Axis.kLeftY.value;
            axes[AxisType.Twist.value] = (byte) XboxController.Axis.kRightX.value;
            axes[AxisType.Throttle.value] = -1;

            hasThrottle = false;
        } else if (getName().equals("Wireless Controller")) {
            System.out.println("Playstation 4 Controller detected!");
            axes[AxisType.Strafe.value] = (byte) PS4Controller.Axis.kLeftX.value;
            axes[AxisType.Forward.value] = (byte) PS4Controller.Axis.kLeftY.value;
            axes[AxisType.Twist.value] = (byte) PS4Controller.Axis.kRightX.value;
            axes[AxisType.Throttle.value] = -1;

            hasThrottle = false;
        } else if (getName().contains("Keyboard")) {
            System.out.println("Keyboard detected!");
            axes[AxisType.Strafe.value] = 0;
            axes[AxisType.Forward.value] = 1;
            axes[AxisType.Twist.value] = 2;
            axes[AxisType.Throttle.value] = 3;

            hasThrottle = true;
        } else if (getName().contains("Xbox Controller")) {
            System.out.println("Xbox Controller detected!");
            axes[AxisType.Strafe.value] = (byte) XboxController.Axis.kLeftX.value;
            axes[AxisType.Forward.value] = (byte) XboxController.Axis.kLeftY.value;
            axes[AxisType.Twist.value] = (byte) XboxController.Axis.kRightX.value;
            axes[AxisType.Throttle.value] = -1;

            hasThrottle = false;
        } else {
            System.out.println("No special controller detected; using default (Xbox) controls.");
            axes[AxisType.Strafe.value] = (byte) XboxController.Axis.kLeftX.value;
            axes[AxisType.Forward.value] = (byte) XboxController.Axis.kLeftY.value;
            axes[AxisType.Twist.value] = (byte) XboxController.Axis.kRightX.value;
            axes[AxisType.Throttle.value] = -1;

            hasThrottle = false;
        }
    }

    public boolean hasThrottle() {
        return hasThrottle;
    }

    public int getStrafeChannel() {
        return axes[AxisType.Strafe.value];
    }

    public void setStrafeChannel(int channel) {
        axes[AxisType.Strafe.value] = (byte) channel;
    }
    
    public int getForwardChannel() {
        return axes[AxisType.Forward.value];
    }

    public void setForwardChannel(int channel) {
        axes[AxisType.Forward.value] = (byte) channel;
    }

    public int getTwistChannel() {
        return axes[AxisType.Twist.value];
    }

    public void setTwistChannel(int channel) {
        axes[AxisType.Twist.value] = (byte) channel;
    }

    public int getThrottleChannel() {
        return axes[AxisType.Throttle.value];
    }

    public void setThrottleChannel(int channel) {
        axes[AxisType.Throttle.value] = (byte) channel;
    }

    public void invertStrafeAxis(boolean bool) {
        axis_inverted[AxisType.Strafe.value] = bool;
    }

    public void invertForwardAxis(boolean bool) {
        axis_inverted[AxisType.Forward.value] = bool;
    }

    public void invertTwistAxis(boolean bool) {
        axis_inverted[AxisType.Twist.value] = bool;
    }

    public void invertThrottleAxis(boolean bool) {
        axis_inverted[AxisType.Throttle.value] = bool;
    }

    public double getStrafeAxis() {
        if (axes[AxisType.Strafe.value] != -1) {
            return getRawAxis(axes[AxisType.Strafe.value]) * (axis_inverted[AxisType.Strafe.value] ? -1 : 1);
        }
        return 0;
    }

    public double getForwardAxis() {
        if (axes[AxisType.Forward.value] != -1) {
            return getRawAxis(axes[AxisType.Forward.value]) * (axis_inverted[AxisType.Forward.value] ? -1 : 1);
        }
        return 0;
    }

    public double getTwistAxis() {
        if (axes[AxisType.Twist.value] != -1) {
            return getRawAxis(axes[AxisType.Twist.value]) * (axis_inverted[AxisType.Twist.value] ? -1 : 1);
        }
        return 0;
    }

    public double getThrottle() {
        if (hasThrottle) {
            return getRawAxis(axes[AxisType.Throttle.value]) * (axis_inverted[AxisType.Throttle.value] ? -1 : 1);
        }
        return 0;
    }

    public String[] getTriggerActions() {
        return triggers.keySet().toArray(new String[triggers.size()]);
    }

    public Trigger getTrigger(String action) {
        return triggers.get(action);
    }

    public void setTrigger(String action, Trigger trigger) {
        triggers.put(action, trigger);
    }

    public void setTrigger(String action, int triggerNumber) {
        triggers.put(action, new JoystickButton(this, triggerNumber));
    }
}
