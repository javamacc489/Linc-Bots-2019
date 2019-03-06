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

public class SetHABLift extends Command {

  double position;
  private double current_position, setpoint, tolerance;

  public SetHABLift(double position) {
    requires(Robot.rHABLift);

    this.position = position;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.rHABLift.setPosition(position);

    setpoint = Robot.rHABLift.getSetpoint();
    tolerance = HABLIFT_TOLERANCE;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    current_position = Robot.rHABLift.getEncoderPosition();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(current_position > (setpoint - tolerance) && current_position < (setpoint + tolerance))
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
