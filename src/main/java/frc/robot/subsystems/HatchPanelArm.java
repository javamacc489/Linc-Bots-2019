/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchPanelArm extends Subsystem {

  private DoubleSolenoid arm_solenoid;
  
  public HatchPanelArm() {
    arm_solenoid = new DoubleSolenoid(RobotMap.HATCHPANELARM_SOLENOID_OUT, RobotMap.HATCHPANELARM_SOLENOID_IN);
  }

  public void arm_out() {
    arm_solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void arm_in() {
    arm_solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public DoubleSolenoid.Value getPosition() {
    return arm_solenoid.get();
  }

  @Override
  public void initDefaultCommand() {
  }
}
