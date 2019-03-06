/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import static frc.robot.RobotConstants.*;

/**
 * Add your docs here.
 */
public class GrabberArm extends Subsystem {
  
  // Motor controllers for the grabber's arm
  private WPI_TalonSRX m_armleft, m_armright;

  public GrabberArm() {
    m_armleft = new WPI_TalonSRX(RobotMap.GRABBER_ARM_LEFT_MOTOR);
    m_armright = new WPI_TalonSRX(RobotMap.GRABBER_ARM_RIGHT_MOTOR);

    m_armleft.setInverted(true);

    m_armleft.configFactoryDefault();
    m_armright.configFactoryDefault();

    m_armleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_armright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // configure the Gain values (PID) for the left and right Grabber Arm motor controllers
		m_armleft.config_kP(0, GRABBERARM_Kp);
		m_armleft.config_kI(0, GRABBERARM_Ki);
    m_armleft.config_kD(0, GRABBERARM_Kd);
    
		m_armright.config_kP(0, GRABBERARM_Kp);
		m_armright.config_kI(0, GRABBERARM_Ki);
    m_armright.config_kD(0, GRABBERARM_Kd);

    // configure the peak output of the motor controllers when in closed-loop mode
    m_armleft.configClosedLoopPeakOutput(0, GRABBERARM_PEAK_OUTPUT);
    m_armright.configClosedLoopPeakOutput(0, GRABBERARM_PEAK_OUTPUT);

    m_armleft.configMotionAcceleration( (int) (10/GRABBERARM_ANGLE_PER_PULSE) );
    m_armleft.configMotionCruiseVelocity( (int) (45/GRABBERARM_ANGLE_PER_PULSE) );
    m_armright.configMotionAcceleration( (int) (10/GRABBERARM_ANGLE_PER_PULSE) );
    m_armright.configMotionCruiseVelocity( (int) (45/GRABBERARM_ANGLE_PER_PULSE) );
  }

  public void putValuesToSmartDashboard() {
    double angle = getAverageEncoderAngle();
    double setpoint = getSetpoint();
    
    SmartDashboard.putNumber("Grabber Arm Angle", angle);
    SmartDashboard.putNumber("Grabber Arm Setpoint", setpoint);
  }

  @Override
  public void initDefaultCommand() {
  }

  public void resetEncoders() {
    m_armleft.setSelectedSensorPosition(0);
    m_armright.setSelectedSensorPosition(0);
  }

  public void setPosition(double degrees) {
    double pulses = degrees / GRABBERARM_ANGLE_PER_PULSE;
    m_armleft.set(ControlMode.MotionMagic, pulses);
    m_armright.set(ControlMode.MotionMagic, pulses);
  }

  public double getSetpoint() {
    double setpoint = m_armleft.getClosedLoopTarget() * GRABBERARM_ANGLE_PER_PULSE;
    return setpoint;
  }

  public double getAverageEncoderAngle() {
    /*
    double left_value = getLeftEncoderAngle();
    double right_value = getRightEncoderAngle();
    double avg = (left_value + right_value) / 2;
    return avg;
    */
    return getRightEncoderAngle();
  }

  public double getLeftEncoderAngle() {
    double pulses = m_armleft.getSelectedSensorPosition();
    double value = pulses * GRABBERARM_ANGLE_PER_PULSE;
    return value;
  }

  public double getRightEncoderAngle() {
    double pulses = m_armright.getSelectedSensorPosition();
    double value = pulses * GRABBERARM_ANGLE_PER_PULSE;
    return value;
  }

  public void stopGrabberArm() {
    m_armleft.stopMotor();
    m_armright.stopMotor();
  }

  public void setTorqueToPeak() {
    m_armleft.configClosedLoopPeakOutput(0, GRABBERARM_PEAK_OUTPUT);
    m_armright.configClosedLoopPeakOutput(0, GRABBERARM_PEAK_OUTPUT);
  }

  public void setTorqueToHolding() {
    m_armleft.configClosedLoopPeakOutput(0, GRABBERARM_HOLDING_OUTPUT);
    m_armright.configClosedLoopPeakOutput(0, GRABBERARM_HOLDING_OUTPUT);
  }
}
