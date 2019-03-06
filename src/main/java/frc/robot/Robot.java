/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.subsystems.CascadingLift;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GrabberArm;
import frc.robot.subsystems.HABLift;
import frc.robot.subsystems.HatchPanelArm;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static UsbCamera camera0;

  public static Drivetrain rDrivetrain;
  public static CascadingLift rCascadingLift;
  public static Grabber rGrabber;
  public static GrabberArm rGrabberArm;
  public static HatchPanelArm rHatchPanelArm;
  public static HABLift rHABLift;

  public static OI m_oi;
  public static Limelight limelight;

  // Compressor on robot
	public static Compressor rCompressor;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    camera0 = CameraServer.getInstance().startAutomaticCapture("USB Camera 0", 0);

    // Create the compressor and tell it to automatically fill with air when low.
		rCompressor = new Compressor(0);
    rCompressor.setClosedLoopControl(true);
    
    rDrivetrain = new Drivetrain();
    rCascadingLift = new CascadingLift();
    rGrabber = new Grabber();
    rGrabberArm = new GrabberArm();
    rHatchPanelArm = new HatchPanelArm();
    rHABLift = new HABLift();

    m_oi = new OI();
    limelight = new Limelight("limelight");

    /*
     * When the robot first turns on and initializes, make sure that the
     * drivetrain's encoders and gyroscope are reset. Make sure that the
     * gyro's PID controller is disabled.
     */
    rDrivetrain.resetEncoders();
    rDrivetrain.resetGyro();
    rDrivetrain.disableGyroPIDController();

    /**
     * When the robot first turns on and initializes, make sure that the
     * CascadingLift's encoder is reset and its position is set to zero.
     */
    rCascadingLift.resetEncoder();
    rCascadingLift.setPosition(0.0);

    /**
     * Make sure the Grabber Arm's encoders are reset when the robot initializes.
     * Also, lock the Grabber Arm into a 'zero' degree angle.
     */
    rGrabberArm.resetEncoders();
    rGrabberArm.setPosition(0.0);

    /**
     * Make sure the Hatch Panel Arm is sucked inward.
     */
    rHatchPanelArm.arm_in();

    rHABLift.resetEncoder();
    rHABLift.setPosition(0.0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Constantly updates the values on the SmartDashboard
    updateSmartDashboard();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    /*
     * When the robot starts autonomous mode, make sure that the
     * drivetrain's encoders and gyroscope are reset. Make sure that the
     * gyro's PID controller is disabled.
     */
    rDrivetrain.resetEncoders();
    rDrivetrain.resetGyro();
    rDrivetrain.disableGyroPIDController();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  // Updates the values on the SmartDashboard
  public void updateSmartDashboard() {
    limelight.putValuesToSmartDashboard();
    rDrivetrain.putValuesToSmartDashboard();
    rCascadingLift.putValuesToSmartDashboard();
    rGrabberArm.putValuesToSmartDashboard();
    rHABLift.putValuesToSmartDashboard();
  }
}
