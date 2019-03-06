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
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

import static frc.robot.RobotConstants.*;

/**
 * Add your docs here.
 */
public class HABLift extends Subsystem {

  private WPI_TalonSRX m_masterhablift;
  private WPI_VictorSPX m_slavehablift;

  public HABLift() {
    m_masterhablift = new WPI_TalonSRX(RobotMap.HABLIFT_LEFT_MOTOR);
    m_slavehablift = new WPI_VictorSPX(RobotMap.HABLIFT_RIGHT_MOTOR);

    m_masterhablift.configFactoryDefault();
    m_slavehablift.configFactoryDefault();

    m_masterhablift.setInverted(true);

    m_slavehablift.follow(m_masterhablift);

    m_masterhablift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    m_masterhablift.config_kP(0, HABLIFT_Kp);
		m_masterhablift.config_kI(0, HABLIFT_Ki);
    m_masterhablift.config_kD(0, HABLIFT_Kd);

    m_masterhablift.configClosedLoopPeakOutput(0, HABLIFT_MAX_SPEED);

    m_masterhablift.configMotionAcceleration( (int) (2/CASCADINGLIFT_DISTANCE_PER_PULSE) );
    m_masterhablift.configMotionCruiseVelocity( (int) (20/CASCADINGLIFT_DISTANCE_PER_PULSE) );
  }

  public void putValuesToSmartDashboard() {
    double encoder_value = getEncoderPosition();
    double setpoint = getSetpoint();

    SmartDashboard.putNumber("HAB Lift Encoder Position", encoder_value);
    SmartDashboard.putNumber("HAB Lift Encoder Setpoint", setpoint);
  }

  public double getEncoderPosition() {
    double pulses = m_masterhablift.getSelectedSensorPosition();
    double value = pulses * HABLIFT_DISTANCE_PER_PULSE;
    return value;
  }

  public void resetEncoder() {
    m_masterhablift.setSelectedSensorPosition(0);
  }

  public void setPosition(double position) {
    double pulses = position / HABLIFT_DISTANCE_PER_PULSE;
    m_masterhablift.set(ControlMode.MotionMagic, pulses);
  }

  public double getSetpoint() {
    if(m_masterhablift.getControlMode() == ControlMode.MotionMagic)
    {
    double pulses = m_masterhablift.getClosedLoopTarget();
    double setpoint = pulses * HABLIFT_DISTANCE_PER_PULSE;
    return setpoint;
    } else
    {
      return 0.0;
    }
  }

  @Override
  public void initDefaultCommand() {
  }
}
