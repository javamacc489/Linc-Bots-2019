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

public class SetCascadingLift extends Command {

  private double height;
  private double current_height, setpoint, tolerance;

  public SetCascadingLift(double height) {
    requires(Robot.rCascadingLift);

    this.height = height;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    current_height = Robot.rCascadingLift.getEncoderValue();

    if(height <= CASCADINGLIFT_RANGE_MAX && height >= CASCADINGLIFT_RANGE_MIN)
    {
      if(height > current_height)
      {
        Robot.rCascadingLift.setMotionAcceleration(2);
        Robot.rCascadingLift.setMotionCruiseVelocity(10);
      } else if (height < current_height)
      {
        Robot.rCascadingLift.setMotionAcceleration(1);
        Robot.rCascadingLift.setMotionCruiseVelocity(3);
      }

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
      if(setpoint < 1.0)
      {
        Robot.rCascadingLift.disablePIDController();
        Robot.rCascadingLift.resetEncoder();
      }
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
