// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.libs.wrappers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** 
 * @author Anish Chandra
 * Wraps the Joystick Object
*/
public class Controller {
    private Joystick joy;
    private double deadband;

    public Controller(int port, double deadband) {
        joy = new Joystick(port);
        this.deadband = deadband;
    }

    public double getAxis(int axis) {
        return joy.getRawAxis(axis);
    }

    public JoystickButton getButton(int btnNum) {
        return new JoystickButton(joy, btnNum);
    }

    public double getLeftJoyX() {
        double joyVal = joy.getX();;
        if(joyVal < deadband && joyVal > -deadband) joyVal = 0;
        return joyVal;
    }

    public double getLeftJoyY() {
        double joyVal = -joy.getY();;
        if(joyVal < deadband && joyVal > -deadband) joyVal = 0;
        return joyVal;
    }

    public double getRightJoyX() {
        double joyVal = joy.getRawAxis(4);
        if(joyVal < deadband && joyVal > -deadband) joyVal = 0;
        return joyVal;
    }

    public double getRightJoyY() {
        double joyVal = -joy.getRawAxis(5);
        if(joyVal < deadband && joyVal > -deadband) joyVal = 0;
        return joyVal;
    }

    public double getLeftTrigger() {
        return joy.getRawAxis(2);
    }

    public double getRightTrigger() {
        return joy.getRawAxis(3);
    }

    public JoystickButton getAButton() {
        return new JoystickButton(joy, 1);
    }

    public JoystickButton getBButton() {
        return new JoystickButton(joy, 2);
    }

    public JoystickButton getXButton() {
        return new JoystickButton(joy, 3);
    }

    public JoystickButton getYButton() {
        return new JoystickButton(joy, 4);
    }

    public JoystickButton getLBButton() {
        return new JoystickButton(joy, 5);
    }
    
    public JoystickButton getRBButton() {
        return new JoystickButton(joy, 6);
    }

    public JoystickButton getSTARTButton() {
        return new JoystickButton(joy, 7);
    }

    public JoystickButton getMENUButton() {
        return new JoystickButton(joy, 8);
    }

    public JoystickButton getLeftStickPress() {
        return new JoystickButton(joy, 9);
    }

    public JoystickButton getRightStickPress() {
        return new JoystickButton(joy, 10);
    }
}
