/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // CAN Bus Connections
  public static final int DRIVETRAIN_FRONT_LEFT_MOTOR = 0;
  public static final int DRIVETRAIN_FRONT_RIGHT_MOTOR = 2;

  public static final int PIGEON_TALON = 1;

  public static final int CASCADINGLIFT_MOTOR = 3;

  public static final int GRABBER_ARM_LEFT_MOTOR = 5;
  public static final int GRABBER_ARM_RIGHT_MOTOR = 6;

  public static final int HABLIFT_LEFT_MOTOR = 7;
  public static final int HABLIFT_RIGHT_MOTOR = 0;

  // PWM Connections
  public static final int GRABBER_TOP_WHEELS_MOTOR = 0;
  public static final int GRABBER_BOTTOM_WHEELS_MOTOR = 1;

  // Pneumatic Connections
  public static final int HATCHPANELARM_SOLENOID_OUT = 0;
  public static final int HATCHPANELARM_SOLENOID_IN = 1;
}
