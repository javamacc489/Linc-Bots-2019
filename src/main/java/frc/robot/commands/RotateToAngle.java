/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RotateToAngle extends Command {
  private double new_angle_setpoint;
  private double zRotation;

  public RotateToAngle(double angle) {
    requires(Robot.rDrivetrain);

    new_angle_setpoint = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.rDrivetrain.setGyroSetpoint(new_angle_setpoint);
    Robot.rDrivetrain.enableGyroPIDController();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    zRotation = Robot.rDrivetrain.getGyroPIDOutput();
    Robot.rDrivetrain.arcadeDrive(0.0, zRotation, false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.rDrivetrain.isGyroOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.rDrivetrain.stop();
    Robot.rDrivetrain.disableGyroPIDController();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
