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
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDrive;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;

import static frc.robot.RobotConstants.*;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // motor controllers
  private WPI_TalonSRX m_masterleft, m_masterright;
  
  private WPI_TalonSRX pigeonTalon;

  // the drivetrain as a whole
  private final DifferentialDrive m_robotDrive;

  // variables to remember the last encoder targets
  private double last_left_encoder_target, last_right_encoder_target;

  // the PigeonIMU connected to a Talon SRX which will function as a gyroscope
  private PigeonIMU pigeonIMU;

  // PID controller to control the gyro (PigeonIMU) connected to a Talon SRX
  private PIDController gyroPIDController;

  public Drivetrain() {
    m_masterleft = new WPI_TalonSRX(RobotMap.DRIVETRAIN_FRONT_LEFT_MOTOR);
    m_masterright = new WPI_TalonSRX(RobotMap.DRIVETRAIN_FRONT_RIGHT_MOTOR);

    pigeonTalon = new WPI_TalonSRX(RobotMap.PIGEON_TALON);

    m_masterleft.configFactoryDefault();
    m_masterright.configFactoryDefault();
    
    pigeonTalon.configFactoryDefault();

    /**
     * m_robotDrive is the drivetrain made up of the left and right master motor
     * controllers.
     * Set the right motor controller to not be inverted because it is inverted
     * using the CTRE library.
     */
    m_robotDrive = new DifferentialDrive(m_masterleft, m_masterright);
    m_robotDrive.setRightSideInverted(false);
    m_robotDrive.setSafetyEnabled(false);

    /*
    * the feedback sensors for both the left and right master drivetrain motor controllers
    * are SRX Mag Encoders
    */
    m_masterleft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
      DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kTimeoutMs);
    m_masterright.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
      DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kTimeoutMs);

    /**
     * Invert the left and right motor controller's encoder.
     */
    m_masterleft.setSensorPhase(true);
    m_masterright.setSensorPhase(true);

    /**
     * Inverts the h-bridge output of the right motor controllers
     */
    m_masterright.setInverted(true);

    // configure the Gain values (PID) for the left and right master drivetrain motor controllers
		m_masterleft.config_kP(DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kGains.kP, DRIVETRAIN_kTimeoutMs);
		m_masterleft.config_kI(DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kGains.kI, DRIVETRAIN_kTimeoutMs);
    m_masterleft.config_kD(DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kGains.kD, DRIVETRAIN_kTimeoutMs);
    
		m_masterright.config_kP(DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kGains.kP, DRIVETRAIN_kTimeoutMs);
		m_masterright.config_kI(DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kGains.kI, DRIVETRAIN_kTimeoutMs);
    m_masterright.config_kD(DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kGains.kD, DRIVETRAIN_kTimeoutMs);

    // configure the peak output of the motor controllers when in closed-loop mode
    m_masterleft.configClosedLoopPeakOutput(DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_MAX_OUTPUT, DRIVETRAIN_kTimeoutMs);
    m_masterright.configClosedLoopPeakOutput(DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_MAX_OUTPUT, DRIVETRAIN_kTimeoutMs);

    m_masterleft.configMotionAcceleration((int)DRIVETRAIN_ACCELERATION_LIMIT);
    m_masterleft.configMotionCruiseVelocity((int)DRIVETRAIN_VELOCITY_LIMIT);
    m_masterright.configMotionAcceleration((int)DRIVETRAIN_ACCELERATION_LIMIT);
    m_masterright.configMotionCruiseVelocity((int)DRIVETRAIN_VELOCITY_LIMIT);

    // encoder targets initialize at zero
    last_left_encoder_target = 0.0;
    last_right_encoder_target = 0.0;

    pigeonIMU = new PigeonIMU(pigeonTalon);

    pigeonIMU.configFactoryDefault();

    gyroPIDController = new PIDController(DRIVETRAIN_GYRO_Kp, DRIVETRAIN_GYRO_Ki, DRIVETRAIN_GYRO_Kd, gyroPIDSource, gyroPIDOutput);

    gyroPIDController.setInputRange(0, 360);
    gyroPIDController.setContinuous();
    gyroPIDController.setAbsoluteTolerance(DRIVETRAIN_GYRO_TOLERANCE);
    gyroPIDController.setOutputRange(-DRIVETRAIN_GYRO_AUTO_SPEED, DRIVETRAIN_GYRO_AUTO_SPEED);
  }

  private final PIDSource gyroPIDSource = new PIDSource() {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
      double pidInput = getGyroAngle();
      return pidInput;
	  }

  };

  private final PIDOutput gyroPIDOutput = new PIDOutput() {

    @Override
    public void pidWrite(double output) {
    }

  };

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new TeleopDrive());
  }

  public void putValuesToSmartDashboard() {
    double masterleft_encoder = getLeftEncoderDistance();
    double masterright_encoder = getRightEncoderDistance();
    double combined_encoders = getAverageEncoderDistance();
    double left_encoder_target = getLeftEncoderTarget();
    double right_encoder_target = getRightEncoderTarget();

    double gyro_angle = getGyroAngle();
    double gyro_setpoint = getGyroSetpoint();
    
    SmartDashboard.putNumber("Master Left Encoder Position (inches)", masterleft_encoder);
    SmartDashboard.putNumber("Master Right Encoder Position (inches)", masterright_encoder);
    SmartDashboard.putNumber("Combined Encoder Distance (inches)", combined_encoders);
    SmartDashboard.putNumber("Left Encoder Target", left_encoder_target);
    SmartDashboard.putNumber("Right Encoder Target", right_encoder_target);
    SmartDashboard.putNumber("Gyro Angle", gyro_angle);
    SmartDashboard.putNumber("Gyro Setpoint", gyro_setpoint);
  }

  // move the robot 'arcade-style' with a joystick
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
    m_robotDrive.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  /*
   * Drive the robot a certain distance in inches.
   * The input parameter is in inches.
   * The function converts the distance in inches back to raw encoder ticks
   * and then passes that value to the motor controllers.
   */
  public void drive_distance(double left_distance_to_drive, double right_distance_to_drive) {
    double left_encoder_ticks = left_distance_to_drive / DRIVETRAIN_DISTANCE_PER_PULSE;
    double right_encoder_ticks = right_distance_to_drive / DRIVETRAIN_DISTANCE_PER_PULSE;

    m_masterleft.set(ControlMode.MotionMagic, left_encoder_ticks);
    m_masterright.set(ControlMode.MotionMagic, right_encoder_ticks);
  }

  // stop the motors, and therefore the robot
  public void stop() {
    m_robotDrive.stopMotor();
  }

  /*
  * Return the average distance traveled between the left and right drivetrain encoders
  * in inches.
  */
  public double getAverageEncoderDistance() {
    // position of left encoder in raw units
    double leftPulse = m_masterleft.getSelectedSensorPosition(DRIVETRAIN_kPIDLoopIdx);
    // position of right encoder in raw units
    double rightPulse = m_masterright.getSelectedSensorPosition(DRIVETRAIN_kPIDLoopIdx);
    // average position of encoders in raw units
    double avgPulse = ( leftPulse + rightPulse ) / 2;

    // distance in inches the robot has traveled
    double avgDist = avgPulse * DRIVETRAIN_DISTANCE_PER_PULSE;
    return avgDist;
  }

  /**
   * Return the distance traveled by the left drivetrain encoder in inches.
   */
  public double getLeftEncoderDistance() {
    // position of left encoder in raw units
    double leftPulse = m_masterleft.getSelectedSensorPosition(DRIVETRAIN_kPIDLoopIdx);
    double distance = leftPulse * DRIVETRAIN_DISTANCE_PER_PULSE;
    return distance;
  }

  /**
   * Return the distance traveled by the right drivetrain encoder in inches.
   */
  public double getRightEncoderDistance() {
    // position of left encoder in raw units
    double rightPulse = m_masterright.getSelectedSensorPosition(DRIVETRAIN_kPIDLoopIdx);
    double distance = rightPulse * DRIVETRAIN_DISTANCE_PER_PULSE;
    return distance;
  }

  /*
   * Returns the target of the left drivetrain encoder in raw encoder units
   */
  public double getRawLeftEncoderTarget() {
    if(m_masterleft.getControlMode() == ControlMode.MotionMagic)
    {
      double raw_target = m_masterleft.getClosedLoopTarget(DRIVETRAIN_kPIDLoopIdx);
      last_left_encoder_target = raw_target;
      return raw_target;
    } else
    {
      return last_left_encoder_target;
    }
  }

  /*
   * Returns the target of the right drivetrain encoder in raw encoder units
   */
  public double getRawRightEncoderTarget() {
    if(m_masterright.getControlMode() == ControlMode.MotionMagic)
    {
      double raw_target = m_masterright.getClosedLoopTarget(DRIVETRAIN_kPIDLoopIdx);
      last_right_encoder_target = raw_target;
      return raw_target;
    } else
    {
      return last_right_encoder_target;
    }
  }

  /*
   * Returns the target of the left drivetrain encoder in inches
   */
  public double getLeftEncoderTarget() {
    double raw_target = getRawLeftEncoderTarget();
    double target = raw_target * DRIVETRAIN_DISTANCE_PER_PULSE;
    return target;
  }

  /*
   * Returns the target of the right drivetrain encoder in inches
   */
  public double getRightEncoderTarget() {
    double raw_target = getRawRightEncoderTarget();
    double target = raw_target * DRIVETRAIN_DISTANCE_PER_PULSE;
    return target;
  }

  // Resets the drivetrain's encoders to a value of zero
  public void resetEncoders() {
    m_masterleft.setSelectedSensorPosition(0, DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kTimeoutMs);
    m_masterright.setSelectedSensorPosition(0, DRIVETRAIN_kPIDLoopIdx, DRIVETRAIN_kTimeoutMs);
  }

  public double getGyroAngle() {
    double[] ypr = new double[3];
    pigeonIMU.getYawPitchRoll(ypr);
    double angle = -ypr[0];
    /**
     * If the absolute value of the angle is greater than 360 degrees, then
     * set the angle back to a value between 0 and 360.
     */
    if(Math.abs(angle) >= 360.0)
    {
      angle = angle % 360.0;
    }
    /**
     * Now that the absolute value of the angle is between 0 and 360, if the angle is
     * negative, add 360 to its value to make it positive.
     */
    if(angle < 0.0)
    {
      angle += 360.0;
    }

    return angle;
  }

  public void resetGyro() {
    pigeonIMU.setYaw(0.0);
  }

  public void enableGyroPIDController() {
    gyroPIDController.enable();
  }

  public void disableGyroPIDController() {
    gyroPIDController.disable();
  }

  public boolean isGyroPIDControllerEnabled() {
    return gyroPIDController.isEnabled();
  }

  public void setGyroSetpoint(double setpoint) {
    gyroPIDController.setSetpoint(setpoint);
  }

  public double getGyroSetpoint() {
    return gyroPIDController.getSetpoint();
  }

  public double getGyroPIDOutput() {
    return gyroPIDController.get();
  }

  public boolean isGyroOnTarget() {
    return gyroPIDController.onTarget();
  }
}
