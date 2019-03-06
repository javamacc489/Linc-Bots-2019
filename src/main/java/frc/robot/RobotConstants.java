/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class RobotConstants {

	public static final double CASCADINGLIFT_CAMERA_VIEW_HEIGHT = 24.0;

	public static final double HAB_PLATFORM_HEIGHT_HIGH = 19.0;

	public static final double HABLIFT_START_CLIMB_HEIGHT_DIFFERENCE = 12.0;

	public static final double CARGO_GRABBING_HEIGHT = 9.0; // 6.5
	public static final double CARGO_GRABBING_ANGLE = 80.0;

	public static final double ROCKET_CARGO_1 = 0.0;
	public static final double ROCKET_CARGO_2 = 26.0;
	public static final double ROCKET_CARGO_3 = 55.0;

	public static final double ROCKET_HATCH_1 = 8.0;
	public static final double ROCKET_HATCH_2 = 38.0;
	public static final double ROCKET_HATCH_3 = 62.0;

	public static final double CARGOSHIP_CARGO_HEIGHT = 17.0;
	public static final double CARGOSHIP_CARGO_ANGLE = 20.0;

	public static final double CARGOSHIP_HATCHPANEL_HEIGHT = 8.0;

	public static final double LOAD_CARGO_APPROACH_HEIGHT = 16.5;

	public static final double LOAD_HATCHPANEL_APPROACH_HEIGHT = 5.0;
	public static final double LOAD_HATCHPANEL_LIFT_HEIGHT = 13.0;
	public static final double DELIVER_HATCHPANEL_HEIGHT_CHANGE = 2.0;

	////////////////////////////////////////////////////////////////////////////////////////


    // constants for the drivetrain's wheels and encoders
	public static final double DRIVETRAIN_WHEEL_DIAMETER = 6.0; // in inches
	public static final double DRIVETRAIN_WHEEL_CIRCUMFERENCE = DRIVETRAIN_WHEEL_DIAMETER * Math.PI;
	public static final double DRIVETRAIN_ENCODER_PULSE_RATE = 4096;
    public static final double 
        DRIVETRAIN_DISTANCE_PER_PULSE = DRIVETRAIN_WHEEL_CIRCUMFERENCE / DRIVETRAIN_ENCODER_PULSE_RATE;
    // constant for depowering the drivetrain speed
    public static final double DRIVETRAIN_DEPOWER_DELTA = 0.4;
    /**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * the drivetrain, use the primary one.
	 */
	public static final int DRIVETRAIN_kPIDLoopIdx = 0;
	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int DRIVETRAIN_kTimeoutMs = 30;
    /**
	 * Gains used in Positon Closed Loop, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
	public static final Gains DRIVETRAIN_kGains = new Gains(1.0, 0.0, 0.075, 0.1, 0, 1.0);
	// tolerance for the drivetrain's encoders in inches (PID control)
	public static final double DRIVETRAIN_ENCODER_TOLERANCE = 0.25;
	// maximum output for the drivetrain's motors when in PID control
	public static final double DRIVETRAIN_MAX_OUTPUT = 1.0;
	public static final double DRIVETRAIN_ON_TARGET_TIME = 0.5;
	// PID values for the gryo
	public static final double DRIVETRAIN_GYRO_Kp = 0.2; // 0.02
	public static final double DRIVETRAIN_GYRO_Ki = 0.0; // 0.002
	public static final double DRIVETRAIN_GYRO_Kd = 0.0; // 0.00002
	// tolerance for the drivetrain's gyroscope in degrees (PID control)
	public static final double DRIVETRAIN_GYRO_TOLERANCE = 1;
	// the absolute value speed the drivetrain (PID control of gyroscope)
	public static final double DRIVETRAIN_GYRO_AUTO_SPEED = 1.0;
	public static final double DRIVETRAIN_VELOCITY_LIMIT = 4/DRIVETRAIN_DISTANCE_PER_PULSE;
	public static final double DRIVETRAIN_ACCELERATION_LIMIT = 4/DRIVETRAIN_DISTANCE_PER_PULSE;

	//////////////////////////////////////////////////////////////////////////////////////////////

	// PID values for the Cascading Lift Subsystem
	public static final double CASCADINGLIFT_Kp = 0.2;
	public static final double CASCADINGLIFT_Ki = 0.0; // 0.001
	public static final double CASCADINGLIFT_Kd = 0.0; // 0.00015
	public static final double CASCADINGLIFT_Kf = 0.0;
	// tolerance for the Cascading Lift in inches (PID control)
	public static final double CASCADINGLIFT_TOLERANCE = 0.25;
	// the absolute value maximum speed of the Cascading Lift
	public static final double CASCADINGLIFT_MAX_SPEED = 0.4;
	// the minimum and maximum input range for the Cascading Lift (PID control) in inches
	public static final double CASCADINGLIFT_RANGE_MIN = 0.0;
	public static final double CASCADINGLIFT_RANGE_MAX = 62.0;
	// constants for the Cascading Lift's wheel and encoder
	// inches per drive motor revolution
	public static final double CASCADINGLIFT_SPROCKET_PERIMETER = 1.273 * Math.PI * 32.75/31.75; // 32.75/31.75 //43.25/43
	public static final double CASCADINGLIFT_STAGE_TWO_TO_STAGE_ONE_RATIO = 2.0/1.0;
	public static final double CASCADINGLIFT_STAGE_TWO_TRAVEL_PER_REVOLUTION = CASCADINGLIFT_SPROCKET_PERIMETER * CASCADINGLIFT_STAGE_TWO_TO_STAGE_ONE_RATIO;
	public static final double CASCADINGLIFT_ENCODER_PULSE_RATE = 4096 * 50;
	public static final double CASCADINGLIFT_DISTANCE_PER_PULSE = CASCADINGLIFT_STAGE_TWO_TRAVEL_PER_REVOLUTION / CASCADINGLIFT_ENCODER_PULSE_RATE;

	////////////////////////////////////////////////////////////////////////////////////////////////
	
	public static final double GRABBER_WHEEL_SPEED = 0.7;

	///////////////////////////////////////////////////////////////////////////////////////////////

	public static final double GRABBERARM_PEAK_OUTPUT = 0.6;
	public static final double GRABBERARM_HOLDING_OUTPUT = 0.5;
	public static final double GRABBERARM_TOLERANCE = 1;
	public static final double GRABBERARM_Kp = 1.0;
	public static final double GRABBERARM_Ki = 0.0;
	public static final double GRABBERARM_Kd = 0.0;

	public static final double GRABBERARM_ROTATION_DPR = 360.0; // degrees per revolution
	public static final double GRABBERARM_SPROCKET_RATIO = 54.0/16.0;
	public static final double GRABBERARM_GEAR_BOX_RATIO = 100.0/1.0;
	public static final double GRABBERARM_ENCODER_PULSE_RATE = 4096;
	public static final double GRABBERARM_ANGLE_PER_PULSE = GRABBERARM_ROTATION_DPR / GRABBERARM_SPROCKET_RATIO / GRABBERARM_ENCODER_PULSE_RATE / GRABBERARM_GEAR_BOX_RATIO;

	///////////////////////////////////////////////////////////////////////////////////////////////

	public static final double HABLIFT_SPROCKET_PERIMETER = 1.273 * Math.PI * 32.75/31.75;
	public static final double HABLIFT_ENCODER_PULSE_RATE = 4096 * 30;
	public static final double HABLIFT_DISTANCE_PER_PULSE = HABLIFT_SPROCKET_PERIMETER / HABLIFT_ENCODER_PULSE_RATE;

	public static final double HABLIFT_TOLERANCE = 0.25;
	public static final double HABLIFT_MAX_SPEED = 0.4;

	public static final double HABLIFT_Kp = 1.0;
	public static final double HABLIFT_Ki = 0.0;
	public static final double HABLIFT_Kd = 0.0;

	////////////////////////////////////////////////////////////////////////////////////////////////


	/**
	 * CONSTANTS FOR LIMELIGHT VISION TARGETING
	 * Considering the targets to have a rectagular perimeter, the height
	 * of the targets is the distance from the ground to the rectangle's center.
	 * All of the target heights are in inches.
	 * The height of the camera is the distance from the ground to roughly the
	 * base of the camera's eye.
	 * The camera's mounting angle is the angle from the camera's eye to its
	 * mounting surface which is parallel to the ground. The angle is measured
	 * in degrees.
	 */
	public static final double VISION_HEIGHT_OF_ROCKET_PORT_TARGET = 36.5;
	public static final double VISION_HEIGHT_OF_ROCKET_HATCH_TARGET = 28.8;
	public static final double VISION_HEIGHT_OF_LOADING_STATION_TARGET = 28.8;
	public static final double VISION_HEIGHT_OF_CARGOSHIP_TARGET = 28.8;
	public static final double VISION_HEIGHT_OF_CAMERA = 10.5;
	public static final double VISION_CAMERA_MOUNTING_ANGLE = 22.5;

	public static final double VISION_KpAim = 0.1;
	public static final double VISION_MIN_AIM_COMMAND = 0.05;
	public static final double VISION_KpDistance = -0.1;

	public static final double VISION_STOP_POINT_FROM_TARGET = 12.0;
}
