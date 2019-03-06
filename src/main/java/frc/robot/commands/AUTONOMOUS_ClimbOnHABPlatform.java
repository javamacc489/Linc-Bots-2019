/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AUTONOMOUS_ClimbOnHABPlatform extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AUTONOMOUS_ClimbOnHABPlatform(double platform_height) {
    // set up everything
    addSequential(new RetractHatchPanelArm());
    addSequential(new SetGrabberArm(90.0));
    addSequential(new SetCascadingLift(platform_height));
    addParallel(new AlignHABLiftToCascadingLift(platform_height));
    addParallel(new DriveGrabberForward());
    addSequential(new SetCascadingLift(0.0));
  }
}
