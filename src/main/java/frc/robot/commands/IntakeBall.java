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

public class IntakeBall extends Command {

  boolean autoFinish;
  boolean first_time;
  Timer timer;

  public IntakeBall() {
    requires(Robot.rGrabber);

    autoFinish = false;
  }

  public IntakeBall(boolean autoFinish) {
    requires(Robot.rGrabber);

    this.autoFinish = autoFinish;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    timer = new Timer();
    first_time = true;
    Robot.rGrabber.intakeBall();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(first_time == true)
    {
      timer.reset();
      timer.start();
      first_time = false;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(autoFinish == false)
    {
      return false;
    } else
    {
      if(timer.get() > 1.5)
      {
        return true;
      } else
      {
        return false;
      }
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.rGrabber.stopGrabber();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
