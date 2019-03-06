/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class DriveDistance extends Command {
  private double distance_to_drive;
  private double new_left_distance_target, new_right_distance_target;
  private double current_left_distance, current_right_distance;
  private double max_left_distance, max_right_distance;
  private double min_left_distance, min_right_distance;
  private Timer timer;
  private boolean onTarget;

  // the distance parameter should be a value in inches
  public DriveDistance(double distance) {
    requires(Robot.rDrivetrain);

    distance_to_drive = distance;

    timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // assume that the robot is not currently at its desired target
    onTarget = false;


    // reset the drivetrain's encoders
    Robot.rDrivetrain.resetEncoders();

    /*
     * Functions return values in inches, so
     * the variables should have values in inches
     */
    current_left_distance = Robot.rDrivetrain.getLeftEncoderDistance();
    current_right_distance = Robot.rDrivetrain.getRightEncoderDistance();

    /*
     * Current distances have values in inches.
     * Distances-to-drive should have values in inches.
     * New distances (the sum of the previous two values) should be
     * values in inches.
     */
    new_left_distance_target = current_left_distance + distance_to_drive;
    new_right_distance_target = current_right_distance + distance_to_drive;

    // maximum and minimum tolerance distances with regards to the target distances
    max_left_distance = new_left_distance_target + RobotConstants.DRIVETRAIN_ENCODER_TOLERANCE;
    min_left_distance = new_left_distance_target - RobotConstants.DRIVETRAIN_ENCODER_TOLERANCE;

    max_right_distance = new_right_distance_target + RobotConstants.DRIVETRAIN_ENCODER_TOLERANCE;
    min_right_distance = new_right_distance_target - RobotConstants.DRIVETRAIN_ENCODER_TOLERANCE;

    // send the new targets to the drivetrain's motor controllers
    Robot.rDrivetrain.drive_distance(new_left_distance_target, new_right_distance_target);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    current_left_distance = Robot.rDrivetrain.getLeftEncoderDistance();
    current_right_distance = Robot.rDrivetrain.getRightEncoderDistance();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // If the drivetrain encoders are within a certain tolerance of their targets...
    if(current_left_distance <= max_left_distance && current_left_distance > min_left_distance
        && current_right_distance <= max_right_distance && current_right_distance > min_right_distance)
    {
      // and the encoders were not previously in that zone...
      if(onTarget == false)
      {
        // note that the encoders are now in that zone, and reset and start the timer.
        onTarget = true;
        timer.reset();
        timer.start();
      }
    } else // If the drivetrain encoders are NOT within the certain tolerance of the target...
    {
      // note that the encoders are NOT in the target zone,
      onTarget = false;
      // and stop stop the timer if it was running.
      timer.stop();
    }

    // If the encoders have been in their target zone for over a half second...
    if(onTarget == true && timer.get() > RobotConstants.DRIVETRAIN_ON_TARGET_TIME)
    {
      // the DriveToDistance command is complete.
      return true;
    } else // If the encoders have left their target zone before a half second has passed...
    {
      // the DriveToDistance command is NOT complete.
      return false;
    }
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
