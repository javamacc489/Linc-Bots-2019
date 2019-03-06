/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Add your docs here.
 */
public class TimedButton extends Button {
    private GenericHID joystick;
    private int buttonNumber;
    double buttonDelay;
    boolean pressed, held;
    Timer delayTimer;

    public TimedButton(GenericHID _joystick, int _buttonNumber, double _delay) {
        joystick = _joystick;
        buttonNumber = _buttonNumber;
        buttonDelay = _delay;
        
        pressed = false;
        held = false;
        delayTimer = new Timer();
    }

    @Override
    public boolean get() {
        checkIfButtonIsHeld();

        if(held == true && delayTimer.get() > buttonDelay)
        {
            return true;
        } else
        {
            return false;
        }
    }

    private void checkIfButtonIsHeld() {
        pressed = joystick.getRawButton(buttonNumber);

        if(pressed)
        {
            if(held == false)
            {
                delayTimer.reset();
                delayTimer.start();
                held = true;
            }
        } else
        {
            held = false;
            delayTimer.stop();
        }
    }
}
