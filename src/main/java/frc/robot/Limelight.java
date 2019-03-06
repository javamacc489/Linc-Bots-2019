/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.RobotConstants.*;

/**
 * Add your docs here.
 */
public class Limelight {
    private NetworkTable table_limelight;
    private NetworkTableEntry tv, tx, ty, ta, camMode, pipeline;

    public Limelight(String table_name) {
        //table_limelight = NetworkTableInstance.getDefault().getTable("limelight");
        table_limelight = NetworkTableInstance.getDefault().getTable(table_name);
        tv = table_limelight.getEntry("tv");
        tx = table_limelight.getEntry("tx");
        ty = table_limelight.getEntry("ty");
        ta = table_limelight.getEntry("ta");
        camMode = table_limelight.getEntry("camMode");
        pipeline = table_limelight.getEntry("pipeline");
    }

    /**
     * Returns if the Limelight has found any valid targets
     */
    public boolean foundTarget() {
        // Get the data from the Network Table (returns either 1 or 0)
        int data = tv.getNumber(0.0).intValue();
        // convert the data into a boolean (1 == true, 0 == false)
        boolean target_found = data == 1? true : false;

        return target_found;
    }

    /**
     * The input parameter of height is the height of the target
     * above ground level.
     */
    public double getDistanceToTarget(double height) {
        // the height of the target
        double h2 = height;
        // height of camera above floor
        double h1 = RobotConstants.VISION_HEIGHT_OF_CAMERA;
        // mounting angle of camera
        double a1 = RobotConstants.VISION_CAMERA_MOUNTING_ANGLE;
        // angle to the target from Limelight center (in degrees)
        double a2 = ty.getDouble(0.0);

        // KNOWN EQUATION: tan(a1+a2) = (h2-h1) / d
        // where d is the distance along a horizontal plane
        double d = (h2-h1) / Math.tan(a1+a2);

        return d;
    }

    public double getSteeringAdjust() {
        double heading_error = tx.getDouble(0.0);
        double steering_adjust = 0.0;

        if(Math.abs(heading_error) > 1.0)
        {
            steering_adjust =
                RobotConstants.VISION_KpAim * heading_error - RobotConstants.VISION_MIN_AIM_COMMAND;
        } else if(Math.abs(heading_error) <= 1.0)
        {
            steering_adjust =
                RobotConstants.VISION_KpAim * heading_error + RobotConstants.VISION_MIN_AIM_COMMAND;
        }
        
        return steering_adjust;
    }

    /*
    public double getDistanceAdjust(double height) {
        double distance_error = getDistanceToTarget(height);
        double distance_adjust = RobotConstants.VISION_KpDistance * distance_error;
        return distance_adjust;
    }

    public double getDistanceAdjust() {
        double distance_error = ty.getDouble(0.0);
        double distance_adjust = RobotConstants.VISION_KpDistance * distance_error;

        return distance_adjust;
    }
    */

    public void putValuesToSmartDashboard() {
        //read values periodically

        /**
         * Get the data from the Network Table (returns either 1 or 0) and convert
         * the data into a boolean (1 == true, 0 == false)
         */
        boolean valid_target = tv.getNumber(0.0).intValue() == 1? true : false;
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        int current_pipeline = getCurrentPipeline();
        double dist_to_rocket_port_target = getDistanceToTarget(VISION_HEIGHT_OF_ROCKET_PORT_TARGET);
        double dist_to_rocket_hatch_target = getDistanceToTarget(VISION_HEIGHT_OF_ROCKET_HATCH_TARGET);
        double dist_to_cargoship_target = getDistanceToTarget(VISION_HEIGHT_OF_CARGOSHIP_TARGET);
        double dist_to_loading_station_target = getDistanceToTarget(VISION_HEIGHT_OF_LOADING_STATION_TARGET);

        // Post to smart dashboard periodically
        SmartDashboard.putBoolean("Valid Target", valid_target);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("Limelight Pipeline", current_pipeline);
        SmartDashboard.putNumber("Distance to Rocket Port Target", dist_to_rocket_port_target);
        SmartDashboard.putNumber("Distance to Rocket Hatch Target", dist_to_rocket_hatch_target);
        SmartDashboard.putNumber("Distance to Cargo Ship Target", dist_to_cargoship_target);
        SmartDashboard.putNumber("Distance to Loading Station Target", dist_to_loading_station_target);
    }

    /**
     * Sets the mode of the Limelight.
     * If the parameter is set to true, then put the Limelight in
     * vision processing mode.
     * If the parameter is set to false, then put the Limelight in
     * driving mode (Increases exposure, disables vision processing).
     */
    public void setCameraMode(boolean mode) {
        // converts a boolean to an integer
        int number = mode? 1 : 0;
        camMode.setNumber(number);
    }

    // Returns the Limelight's current pipeline
    public int getCurrentPipeline() {
        int current_pipeline = pipeline.getNumber(0.0).intValue();
        return current_pipeline;
    }

    // Sets the Limelight to a new pipeline
    public void setCurrentPipeline(int new_pipeline) {
        pipeline.setNumber(new_pipeline);
    }

}
