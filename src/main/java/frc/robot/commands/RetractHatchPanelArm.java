/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class RetractHatchPanelArm extends Command {

  boolean state;
  Timer timer;

  public RetractHatchPanelArm() {
    requires(Robot.rHatchPanelArm);

    timer = new Timer();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    state = false;
    Robot.rHatchPanelArm.arm_in();
    timer.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(state == false)
    {
      if(Robot.rHatchPanelArm.getPosition() == Value.kReverse)
      {
        state = true;
        timer.start();
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return state == true && timer.get() > 0.5;
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
