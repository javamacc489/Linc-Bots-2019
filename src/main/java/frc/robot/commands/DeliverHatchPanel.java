/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

import static frc.robot.RobotConstants.*;

public class DeliverHatchPanel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public DeliverHatchPanel(double height) {
    addParallel(new RetractHatchPanelArm());
    addParallel(new SetGrabberArm(0.0));
    addParallel(new SetCascadingLift(height));
    // drive to the target (sequential)
    // addSequential(new PunchHatchPanelArm());
    // addSequential(new SetCascadingLift(height - DELIVER_HATCHPANEL_HEIGHT_CHANGE));
    // addSequential(new RetractHatchPanelArm());
  }
}
