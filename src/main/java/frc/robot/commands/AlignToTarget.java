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

public class AlignToTarget extends Command {
  private boolean finished = false;

  private double steering_adjust;
  private double left_command, right_command;
  private double height_of_target;
  private double stopping_distance = VISION_STOP_POINT_FROM_TARGET;
  private double distance_to_target, current_left_distance, current_right_distance;

  private double current_distance;
  private double distance_error;
  private double distance_adjust;

  public AlignToTarget(double target_height) {
    requires(Robot.rDrivetrain);
    this.height_of_target = target_height;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.rDrivetrain.resetEncoders();
    /*
    if(Robot.limelight.foundTarget() == false)
    {
      finished = true;
    }
    */
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("Running Target Alignment!!");

    /*
    if(Robot.limelight.foundTarget())
    {
      distance_to_target = Robot.limelight.getDistanceToTarget(height_of_target) - stopping_distance;

      left_command = current_left_distance + distance_to_target;
      right_command = current_right_distance + distance_to_target;
      
      steering_adjust = Robot.limelight.getSteeringAdjust();
      
      left_command += steering_adjust;
      right_command -= steering_adjust;

      Robot.rDrivetrain.drive_distance(left_command, right_command);
    }
    */
    if(Robot.limelight.foundTarget())
    {
      distance_to_target = current_distance = Robot.limelight.getDistanceToTarget(height_of_target);
      distance_error = stopping_distance - current_distance;
      distance_adjust = VISION_KpDistance * distance_error;
      if(distance_adjust > 0.7)
      {
        distance_adjust = 0.7;
      }
      Robot.rDrivetrain.arcadeDrive(distance_adjust, 0.0, false);
    } else
    {
      Robot.rDrivetrain.arcadeDrive(0.0, 0.0, false);
    }

    current_left_distance = Robot.rDrivetrain.getLeftEncoderDistance();
    current_right_distance = Robot.rDrivetrain.getRightEncoderDistance();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    /*
    if(Math.abs(distance_to_target - current_left_distance) < 1 &&
        Math.abs(distance_to_target - current_right_distance) < 1)
    {
      finished = true;
    }

    return finished;
    */
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.rDrivetrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
