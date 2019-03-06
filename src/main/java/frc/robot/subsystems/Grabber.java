/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import static frc.robot.RobotConstants.*;

/**
 * Add your docs here.
 */
public class Grabber extends Subsystem {

  // Motor controllers for the grabber's wheels
  private Spark m_topwheels, m_bottomwheels;

  public Grabber() {
    m_topwheels = new Spark(RobotMap.GRABBER_TOP_WHEELS_MOTOR);
    m_bottomwheels = new Spark(RobotMap.GRABBER_BOTTOM_WHEELS_MOTOR);
  }

  @Override
  public void initDefaultCommand() {
  }

  public void intakeBall() {
    m_topwheels.set(GRABBER_WHEEL_SPEED);
    m_bottomwheels.set(GRABBER_WHEEL_SPEED);
  }

  public void outputBall() {
    m_topwheels.set(-GRABBER_WHEEL_SPEED);
    m_bottomwheels.set(-GRABBER_WHEEL_SPEED);
  }

  public void driveforward() {
    m_topwheels.set(1.0);
    m_bottomwheels.set(-1.0);
  }

  public void stopGrabber() {
    m_topwheels.stopMotor();
    m_bottomwheels.stopMotor();
  }
}
