/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class SetGrabberArm extends Command {
  
  private double degrees;
  private double current_angle, setpoint, tolerance;

  public SetGrabberArm(double degrees) {
    requires(Robot.rGrabberArm);
    
    this.degrees = degrees;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(degrees >= 0.0 && degrees <= 90.0)
    {
      Robot.rGrabberArm.setPosition(degrees);
    }

    setpoint = Robot.rGrabberArm.getSetpoint();
    tolerance = RobotConstants.GRABBERARM_TOLERANCE;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    current_angle = Robot.rGrabberArm.getAverageEncoderAngle();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(current_angle > (setpoint - tolerance) && current_angle < (setpoint + tolerance))
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
