/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AlignHABLiftToCascadingLift extends Command {

  private double cascadinglift_position;
  private double cascadinglift_start_position;
  private double following_position;

  public AlignHABLiftToCascadingLift(double cascadinglift_start_position) {
    requires(Robot.rHABLift);

    this.cascadinglift_start_position = cascadinglift_start_position;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    cascadinglift_position = Robot.rCascadingLift.getEncoderValue();
    following_position = cascadinglift_start_position - cascadinglift_position;
    Robot.rHABLift.setPosition(following_position);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
