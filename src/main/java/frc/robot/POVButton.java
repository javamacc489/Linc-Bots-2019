/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Add your docs here.
 */
public class POVButton extends Button {
    private GenericHID joystick;
    private int degrees;

    public POVButton(GenericHID _joystick, int _degrees) {
        joystick = _joystick;
        degrees = _degrees;
    }

    @Override
    public boolean get() {
        return joystick.getPOV() == degrees;
    }
}
