/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ManualDeliverHatchPanel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ManualDeliverHatchPanel() {
    addParallel(new RetractHatchPanelArm());
    addSequential(new SetGrabberArm(0.0));
    addSequential(new PunchHatchPanelArm());
    addSequential(new SetCascadingLiftToDeliverHatchPanelVariableHeight());
    addSequential(new RetractHatchPanelArm());
  }
}
