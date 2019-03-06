/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

import static frc.robot.RobotConstants.*;

public class SetRobotToNormalDriveMode extends CommandGroup {
  /**
   * Add your docs here.
   */
  public SetRobotToNormalDriveMode() {
    addParallel(new RetractHatchPanelArm());
    addParallel(new SetGrabberArm(0.0));
    addParallel(new SetHABLift(0.0));
    addParallel(new SetCascadingLift(CASCADINGLIFT_CAMERA_VIEW_HEIGHT));
  }
}
