/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.AUTONOMOUS_ClimbOnHABPlatform;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.LoadCargo;
import frc.robot.commands.LoadHatchPanel;
import frc.robot.commands.ManualDeliverHatchPanel;
import frc.robot.commands.ManualLoadHatchPanel;
import frc.robot.commands.OutputBall;
import frc.robot.commands.DeliverCargo;
import frc.robot.commands.DeliverHatchPanel;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.PunchHatchPanelArm;
import frc.robot.commands.RetractHatchPanelArm;
import frc.robot.commands.SetCascadingLift;
import frc.robot.commands.SetHABLift;
import frc.robot.commands.SetRobotToGrabbingBallMode;
import frc.robot.commands.SetRobotToNormalDriveMode;

import static frc.robot.RobotConstants.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick usbcontroller0, usbcontroller1;
  
  private Button buttonA, buttonB, buttonX, buttonY;
  private TimedButton timedButtonB;
  private Button buttonLB, buttonRB;
  private POVButton button_UP, button_DOWN;
  private Button button1, button2, button3, button4, button5,
                  button6, button7, button8, button9, button10,
                  button11, button12;

  public OI() {
    usbcontroller0 = new Joystick(0);
    usbcontroller1 = new Joystick(1);

    ////////////////////////////////////////////

    buttonA = new JoystickButton(usbcontroller0, 1);
    buttonB = new JoystickButton(usbcontroller0, 2);
    buttonX = new JoystickButton(usbcontroller0, 3);
    buttonY = new JoystickButton(usbcontroller0, 4);

    timedButtonB = new TimedButton(usbcontroller0, 2, 1.5);

    buttonLB = new JoystickButton(usbcontroller0, 5);
    buttonRB = new JoystickButton(usbcontroller0, 6);

    button_UP = new POVButton(usbcontroller0, 0);
    button_DOWN = new POVButton(usbcontroller0, 180);

    button1 = new JoystickButton(usbcontroller1, 1);
    button2 = new JoystickButton(usbcontroller1, 2);
    button3 = new JoystickButton(usbcontroller1, 3);
    button4 = new JoystickButton(usbcontroller1, 4);
    button5 = new JoystickButton(usbcontroller1, 5);
    button6 = new JoystickButton(usbcontroller1, 6);
    button7 = new JoystickButton(usbcontroller1, 7);
    button8 = new JoystickButton(usbcontroller1, 8);
    button9 = new JoystickButton(usbcontroller1, 9);
    button10 = new JoystickButton(usbcontroller1, 10);
    button11 = new JoystickButton(usbcontroller1, 11);
    button12 = new JoystickButton(usbcontroller1, 12);

    /////////////////////////////////////////////////////////////////////////

    buttonA.whenPressed(new SetRobotToGrabbingBallMode());
    buttonB.whenPressed(new SetRobotToNormalDriveMode());

    timedButtonB.whenPressed(new TeleopDrive());

    buttonX.whenPressed(new ManualLoadHatchPanel());
    buttonY.whenPressed(new ManualDeliverHatchPanel());

    buttonLB.whileHeld(new OutputBall());
    buttonRB.whileHeld(new IntakeBall());

    button_UP.whileHeld(new PunchHatchPanelArm());
    button_UP.whenReleased(new RetractHatchPanelArm());

    ///////////////////////////////

    button1.whenPressed(new DeliverCargo(ROCKET_CARGO_1));
    button2.whenPressed(new DeliverCargo(ROCKET_CARGO_2));
    button3.whenPressed(new DeliverCargo(ROCKET_CARGO_3));
    button4.whenPressed(new DeliverCargo(CARGOSHIP_CARGO_HEIGHT, CARGOSHIP_CARGO_ANGLE));
    button5.whenPressed(new LoadCargo());

    button6.whenPressed(new DeliverHatchPanel(ROCKET_HATCH_1));
    button7.whenPressed(new DeliverHatchPanel(ROCKET_HATCH_2));
    button8.whenPressed(new DeliverHatchPanel(ROCKET_HATCH_3));
    button9.whenPressed(new DeliverHatchPanel(CARGOSHIP_HATCHPANEL_HEIGHT));
    button10.whenPressed(new LoadHatchPanel());
    
    // button11.whenPressed(new SetCascadingLift(ROCKET_CARGO_2));
    // button11.whenReleased(new SetCascadingLift(ROCKET_CARGO_1));
    // button11.whenPressed(new SetHABLift(15.0));
    // button11.whenReleased(new SetHABLift(0.0));
    button11.whenPressed(new DriveDistance(24.0));
    // button11.whenPressed(new AlignToTarget(VISION_HEIGHT_OF_ROCKET_PORT_TARGET));

    button12.whenPressed(new AUTONOMOUS_ClimbOnHABPlatform(HAB_PLATFORM_HEIGHT_HIGH));
  }

  public void setController0Rumble(double intensity) {
    usbcontroller0.setRumble(RumbleType.kLeftRumble, intensity);
    usbcontroller0.setRumble(RumbleType.kRightRumble, intensity);
  }
}
