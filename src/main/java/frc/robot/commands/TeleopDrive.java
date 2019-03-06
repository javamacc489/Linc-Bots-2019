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
import static frc.robot.RobotConstants.*;

public class TeleopDrive extends Command {
  /*
   * Two variables that store the current values from a controller's joysticks.
   */
  double joystickValueX, joystickValueZ;

  /*
   * Two variables that represent the robot's motion along a flat plain,
   * and its twisting motion from its center
   */
  double xSpeed, zRotation;

  /**
   * Angle values for the gyro's PID controller to use.
   */
  double currentAngle, setpointAngle;

  // percentage amount to decrease the values of the xSpeed and zRotation
  double depowerval;

  /*
   * Determines if the input values from the controlling usb controller
   * will be squared to decrease the sensitivity (value of 0.5 would become 0.25).
   */
  boolean squareInputs;

  // Timer to control the driving controller's rumble
  Timer rumbleTimer;

  public TeleopDrive() {
    requires(Robot.rDrivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // make sure to square the inputs from the usb controller
    squareInputs = true;

    // make sure the drivetrain is not being controlled by the gyro's PID controller
    Robot.rDrivetrain.disableGyroPIDController();
    // Set the gyro's (currently disabled) setpoint to the angle that the robot is facing.
    currentAngle = Robot.rDrivetrain.getGyroAngle();
    setpointAngle = currentAngle;

    /**
     * Rumble the main driving controller for a short while for some haptic feedback
     * to represent that the driver is back in manual control of the robot.
     */
    Robot.m_oi.setController0Rumble(0.5);
    rumbleTimer = new Timer();
    rumbleTimer.reset();
    rumbleTimer.start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // Stop the controller's rumble after a short while
    if(rumbleTimer.get() > 1)
    {
      Robot.m_oi.setController0Rumble(0.0);
      rumbleTimer.stop();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    
    /*
     * On a standard usb game controller, axis 1 is the y value of the left stick.
     * The computer usually gives a negative value for pushing forward on the y value of a joystick,
     * so multiply the input value by -1 for simplicity of programming motion.
     */
    joystickValueX = Robot.m_oi.usbcontroller0.getRawAxis(1) * -1;

    // if the joystick is only slightly tilted, don't read the input
    if (Math.abs(joystickValueX) < 0.10) { joystickValueX = 0.0; }

    // On a standard usb game controller, axis 4 is the x value of the right stick.
    joystickValueZ = Robot.m_oi.usbcontroller0.getRawAxis(4);

    // if the joystick is only slightly tilted, don't read the input
    if (Math.abs(joystickValueZ) < 0.10) { joystickValueZ = 0.0; }

    //////////////////////////////////////////////////////////////////////////////////////////////////

    // If the  Z-joystick is being moved...
    if(joystickValueZ != 0.0)
    {
      // ...make sure the gyro's PID Controller is disabled.
      Robot.rDrivetrain.disableGyroPIDController();
      // Set the gyro's (currently disabled) setpoint to the angle that the robot is facing.
      currentAngle = Robot.rDrivetrain.getGyroAngle();
      setpointAngle = currentAngle;
      // ...set the gyro's setpoint to be the last calculated angle the robot was facing.
      //Robot.rDrivetrain.setGyroSetpoint(setpointAngle);
    } else // If the Z-joystick is NOT being moved... (value equal to 0.0)
    {
      /*
      // ...set the gyro's setpoint to be the last calculated angle the robot was facing.
      Robot.rDrivetrain.setGyroSetpoint(setpointAngle);
      */
      /**
       * Only enable the gyros' PID Controller if the X-joystick IS being moved.
       * (Disable the PID Controller if both joysticks are NOT being moved.)
       */
      if(joystickValueX != 0.0)
      {
        Robot.rDrivetrain.enableGyroPIDController();
      } else
      {
        Robot.rDrivetrain.disableGyroPIDController();
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////

    /*
     * if the depowering trigger is held all the way down and the depower delta is 0.4:
     * 1. the trigger's value will be 1
     * 2. 1 * 0.4 = 0.4
     * 3. 1 - 0.4 = 0.6
     * 4. the depowerval would equal 0.6 (the robot will move 60% of its original value)

     * if the depowering trigger is not held down at all, its value would be 0
     * 1. the trigger's value would be 0
     * 2. 0 * 0.4 = 0
     * 3. 1 - 0 = 1
     * 4. the depowerval would equal 1 (the robot will move 100% of its original value)
     */
    // on a standard usb game controller, axis 2 is the left trigger
    depowerval = 1 - Robot.m_oi.usbcontroller0.getRawAxis(2) * DRIVETRAIN_DEPOWER_DELTA;

    // Set the xSpeed to be based off the X-joystick's value.
    xSpeed = joystickValueX * depowerval;

    /**
     * THIS IS TO KILL ALL PID CONTROL
     */
    Robot.rDrivetrain.disableGyroPIDController();

    // If the gyro's PID Controller is enabled...
    if(Robot.rDrivetrain.isGyroPIDControllerEnabled())
    {
      // ...set the zRotation to the PID Controller's output.
      zRotation = Robot.rDrivetrain.getGyroPIDOutput();
    } else // If the PID Controller is NOT enabled...
    {
      // ...set the zRotation to be based off the Z-joystick's value.
      zRotation = joystickValueZ * depowerval;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////
    
    /*
    * Move the robot by its new x and z values (which incorporates how much to depower).
    * Square the joystick inputs to make sure the they're not as 'touchy'.
    */
    Robot.rDrivetrain.arcadeDrive(xSpeed, zRotation, squareInputs);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.rDrivetrain.disableGyroPIDController();
    Robot.rDrivetrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
