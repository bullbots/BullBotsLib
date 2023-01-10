// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1891.common.control;

import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** A controller that detects the device plugged in, and uses the correct inputs to control the robot. */
public class SmartController extends GenericHID {
    protected static final boolean[] default_axis_inverted = new boolean[] {
        false,
        false,
        false,
        false
    };

    public enum AxisType {
        kStrafe(0),
        kForward(1),
        kTwist(2),
        kThrottle(3);

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
    private final byte[] m_axes = new byte[AxisType.values().length];
    private final boolean[] m_axis_inverted = new boolean[AxisType.values().length];

    private final HashMap<String, Trigger> m_triggers = new HashMap<>();

    private boolean hasThrottle;

    public SmartController(int port) {
        super(port);

        System.out.printf("Info: SmartController(%d)\n"+
                        "\tName: \"%s\"\n"+
                        "\tType: HIDType.%s\n", port, getName(), getType());
        configure();
    }

    public void configure() {
        System.out.printf("Info: SmartController(%d) ", getPort());

        m_axis_inverted[AxisType.kStrafe.value] = default_axis_inverted[AxisType.kStrafe.value];
        m_axis_inverted[AxisType.kForward.value] = default_axis_inverted[AxisType.kForward.value];
        m_axis_inverted[AxisType.kTwist.value] = default_axis_inverted[AxisType.kTwist.value];
        m_axis_inverted[AxisType.kThrottle.value] = default_axis_inverted[AxisType.kThrottle.value];

        if (getName().equals("Logitech Extreme 3D")) {
            System.out.println("Logitech Extreme 3D detected!");
            m_axes[AxisType.kStrafe.value] = 0;
            m_axes[AxisType.kForward.value] = 1;
            m_axes[AxisType.kTwist.value] = 2;
            m_axes[AxisType.kThrottle.value] = 3;

            hasThrottle = true;
        } else if (getName().equals("Logitech Attack 3")) {
            System.out.println("Logitech Attack 3 detected!");
            m_axes[AxisType.kStrafe.value] = -1;
            m_axes[AxisType.kForward.value] = 1;
            m_axes[AxisType.kTwist.value] = 0;
            m_axes[AxisType.kThrottle.value] = 3;

            hasThrottle = true;
        } else if (getName().contains("Logitech Dual Action")) {
            System.out.println("Logitech Dual Action detected!");
            m_axes[AxisType.kStrafe.value] = (byte) XboxController.Axis.kLeftX.value;
            m_axes[AxisType.kForward.value] = (byte) XboxController.Axis.kLeftY.value;
            m_axes[AxisType.kTwist.value] = (byte) XboxController.Axis.kRightX.value;
            m_axes[AxisType.kThrottle.value] = -1;

            hasThrottle = false;
        } else if (getName().equals("Wireless Controller")) {
            System.out.println("Playstation 4 Controller detected!");
            m_axes[AxisType.kStrafe.value] = (byte) PS4Controller.Axis.kLeftX.value;
            m_axes[AxisType.kForward.value] = (byte) PS4Controller.Axis.kLeftY.value;
            m_axes[AxisType.kTwist.value] = (byte) PS4Controller.Axis.kRightX.value;
            m_axes[AxisType.kThrottle.value] = -1;

            hasThrottle = false;
        } else if (getName().contains("Keyboard")) {
            System.out.println("Keyboard detected!");
            m_axes[AxisType.kStrafe.value] = 0;
            m_axes[AxisType.kForward.value] = 1;
            m_axes[AxisType.kTwist.value] = 2;
            m_axes[AxisType.kThrottle.value] = 3;

            hasThrottle = true;
        } else if (getName().contains("Xbox Controller")) {
            System.out.println("Xbox Controller detected!");
            m_axes[AxisType.kStrafe.value] = (byte) XboxController.Axis.kLeftX.value;
            m_axes[AxisType.kForward.value] = (byte) XboxController.Axis.kLeftY.value;
            m_axes[AxisType.kTwist.value] = (byte) XboxController.Axis.kRightX.value;
            m_axes[AxisType.kThrottle.value] = -1;

            hasThrottle = false;
        } else {
            System.out.println("No special controller detected; using default (Xbox) controls.");
            m_axes[AxisType.kStrafe.value] = (byte) XboxController.Axis.kLeftX.value;
            m_axes[AxisType.kForward.value] = (byte) XboxController.Axis.kLeftY.value;
            m_axes[AxisType.kTwist.value] = (byte) XboxController.Axis.kRightX.value;
            m_axes[AxisType.kThrottle.value] = -1;

            hasThrottle = false;
        }
    }

    public boolean hasThrottle() {
        return hasThrottle;
    }

    public int getStrafeChannel() {
        return m_axes[AxisType.kStrafe.value];
    }

    public void setStrafeChannel(int channel) {
        m_axes[AxisType.kStrafe.value] = (byte) channel;
    }
    
    public int getForwardChannel() {
        return m_axes[AxisType.kForward.value];
    }

    public void setForwardChannel(int channel) {
        m_axes[AxisType.kForward.value] = (byte) channel;
    }

    public int getTwistChannel() {
        return m_axes[AxisType.kTwist.value];
    }

    public void setTwistChannel(int channel) {
        m_axes[AxisType.kTwist.value] = (byte) channel;
    }

    public int getThrottleChannel() {
        return m_axes[AxisType.kThrottle.value];
    }

    public void setThrottleChannel(int channel) {
        m_axes[AxisType.kThrottle.value] = (byte) channel;
    }

    public void invertStrafeAxis(boolean bool) {
        m_axis_inverted[AxisType.kStrafe.value] = bool;
    }

    public void invertForwardAxis(boolean bool) {
        m_axis_inverted[AxisType.kForward.value] = bool;
    }

    public void invertTwistAxis(boolean bool) {
        m_axis_inverted[AxisType.kTwist.value] = bool;
    }

    public void invertThrottleAxis(boolean bool) {
        m_axis_inverted[AxisType.kThrottle.value] = bool;
    }

    public double getStrafeAxis() {
        if (m_axes[AxisType.kStrafe.value] != -1) {
            return getRawAxis(m_axes[AxisType.kStrafe.value]) * (m_axis_inverted[AxisType.kStrafe.value] ? -1 : 1);
        }
        return 0;
    }

    public double getForwardAxis() {
        if (m_axes[AxisType.kForward.value] != -1) {
            return getRawAxis(m_axes[AxisType.kForward.value]) * (m_axis_inverted[AxisType.kForward.value] ? -1 : 1);
        }
        return 0;
    }

    public double getTwistAxis() {
        if (m_axes[AxisType.kTwist.value] != -1) {
            return getRawAxis(m_axes[AxisType.kTwist.value]) * (m_axis_inverted[AxisType.kTwist.value] ? -1 : 1);
        }
        return 0;
    }

    public double getThrottle() {
        if (hasThrottle) {
            return getRawAxis(m_axes[AxisType.kThrottle.value]) * (m_axis_inverted[AxisType.kThrottle.value] ? -1 : 1);
        }
        return 0;
    }

    public String[] getTriggerActions() {
        return m_triggers.keySet().toArray(new String[m_triggers.size()]);
    }

    public Trigger getTrigger(String action) {
        return m_triggers.get(action);
    }

    public void setTrigger(String action, Trigger trigger) {
        m_triggers.put(action, trigger);
    }

    public void setTrigger(String action, int triggerNumber) {
        m_triggers.put(action, new JoystickButton(this, triggerNumber));
    }
}
