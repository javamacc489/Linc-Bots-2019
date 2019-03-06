/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import static frc.robot.RobotConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * Add your docs here.
 */
public class CascadingLift extends Subsystem {

  // motor controller
  private WPI_TalonSRX m_cascadinglift;
  
  public CascadingLift() {

    m_cascadinglift = new WPI_TalonSRX(RobotMap.CASCADINGLIFT_MOTOR);

    m_cascadinglift.configFactoryDefault();
    m_cascadinglift.setInverted(true);
    m_cascadinglift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_cascadinglift.setSensorPhase(false);

    m_cascadinglift.config_kP(0, CASCADINGLIFT_Kp);
    m_cascadinglift.config_kI(0, CASCADINGLIFT_Ki);
    m_cascadinglift.config_kD(0, CASCADINGLIFT_Kd);
    //m_cascadinglift.config_kF(0, CASCADINGLIFT_Kf);

    m_cascadinglift.configClosedLoopPeakOutput(0, CASCADINGLIFT_MAX_SPEED);

    m_cascadinglift.configMotionAcceleration( (int) (2/CASCADINGLIFT_DISTANCE_PER_PULSE) );
    m_cascadinglift.configMotionCruiseVelocity( (int) (10/CASCADINGLIFT_DISTANCE_PER_PULSE) ); //10 // 2.8
  }

  public void putValuesToSmartDashboard() {
    double elevation = getEncoderValue();
    double setpoint = getSetpoint();

    SmartDashboard.putNumber("Cascading Lift Elevation (encoder)", elevation);
    SmartDashboard.putNumber("Cascading Lift Setpoint", setpoint);
  }

  public void setMotionCruiseVelocity(int velocity) {
    velocity = (int) (velocity/CASCADINGLIFT_DISTANCE_PER_PULSE);
    m_cascadinglift.configMotionCruiseVelocity(velocity);
  }

  public void setMotionAcceleration(int acceleration) {
    acceleration = (int) (acceleration/CASCADINGLIFT_DISTANCE_PER_PULSE);
    m_cascadinglift.configMotionAcceleration(acceleration);
  }

  public void disablePIDController() {
    m_cascadinglift.set(ControlMode.PercentOutput, 0.0);
  }

  public void setPosition(double height) {
    double pulses = height / CASCADINGLIFT_DISTANCE_PER_PULSE;

    m_cascadinglift.set(ControlMode.MotionMagic, pulses);
  }

  public double getSetpoint() {
    double value = m_cascadinglift.getClosedLoopTarget() * CASCADINGLIFT_DISTANCE_PER_PULSE;
    return value;
  }

  public void resetEncoder() {
    m_cascadinglift.setSelectedSensorPosition(0);
  }

  public double getRawEncoderValue() {
    double value = m_cascadinglift.getSelectedSensorPosition();
    return value;
  }

  /**
   * Returns the position of the encoder in inches.
   */
  public double getEncoderValue() {
    // position of encoder in raw units
    double pulse = m_cascadinglift.getSelectedSensorPosition();
    // position of encoder in inches
    double value = pulse * CASCADINGLIFT_DISTANCE_PER_PULSE;
    return value;
  }

  /**
   * Stops the Cascading Lift's motor.
   */
  public void stop() {
    m_cascadinglift.stopMotor();
  }

  @Override
  public void initDefaultCommand() {
  }
}
