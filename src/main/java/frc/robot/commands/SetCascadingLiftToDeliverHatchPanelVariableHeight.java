/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import static frc.robot.RobotConstants.*;

public class SetCascadingLiftToDeliverHatchPanelVariableHeight extends Command {

  private double current_height, height, setpoint, tolerance;

  public SetCascadingLiftToDeliverHatchPanelVariableHeight() {
    requires(Robot.rCascadingLift);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    height = Robot.rCascadingLift.getSetpoint() - DELIVER_HATCHPANEL_HEIGHT_CHANGE;

    if(height <= CASCADINGLIFT_RANGE_MAX && height >= CASCADINGLIFT_RANGE_MIN)
    {
      Robot.rCascadingLift.setPosition(height);
    }

    setpoint = Robot.rCascadingLift.getSetpoint();
    tolerance = CASCADINGLIFT_TOLERANCE;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    current_height = Robot.rCascadingLift.getEncoderValue();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(current_height > (setpoint - tolerance) && current_height < (setpoint + tolerance))
    {
      return true;
    } else
    {
      return false;
    }
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
