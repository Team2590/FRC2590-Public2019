/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Limelight camera class for vision processing Uses network tables relayed from
 * the camera to track retroreflective targets
 * 
 * Access the Limelight web pipeline via: http://10.25.90.11:5801 
 * Access the Limelight camera feed via: http://10.25.90.11:5800
 * Access the Limelight IP camera in GRIP via: http://10.25.90.11:5802
 */
public class Limelight extends Subsystem implements LimelightSettings {

  // singleton
  private static Limelight limelightInstance = null;

  public static Limelight getLimelightInstance() {
    if (limelightInstance == null) {
      limelightInstance = new Limelight();
    }
    return limelightInstance;
  }

  // network table from limelight camera
  NetworkTable table;

  // target coordinates and values
  double tx, ty, tz, tv;

  // used for auto-aligning process
  double kPAim = -0.1;
  double kPDistance = -0.1;
  double min_aim_command = 0.05;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = 0.0; // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    ty = 0.0; // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    tz = 0.0; // distance from the camera to the target
    tv = 0.0; // whether the limelight has any valid targets (0 or 1)
  }

  public void update() {
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    tv = table.getEntry("tv").getDouble(0);
  }

  /**
   * Calculates the angle to the retro-reflective target
   * 
   * @return the vertical angle from the camera to the target
   */
  public double verticalAngleToTarget() {
    if (tv == 1) {
      return ty;
    } else {
      return 0.0;
    }
  }

  /**
   * Calculates the angle to the retro-reflective target
   * 
   * @return the horizontal angle from the camera to the target
   */
  public double horizontalAngleToTarget() {
    if (tv == 1) {
      return tx;
    } else {
      return 0.0;
    }
  }

  /**
   * Calculates the distance to the retro-reflective target
   * 
   * @return the horizontal distance from the camera to the target
   */
  public double distanceToTarget() {
    return (HATCH_TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(CAMERA_ANGLE + verticalAngleToTarget());
  }

  /**
   * Adjusts steering based on the horizontal offset
   * 
   * @return the value to adjust steering by to get it in line with target
   */
  public double adjustSteering() {

    double headingError = (double) -tx;
    double steeringAdjust = 0.0f;

    if (tx > 1.0) {
      steeringAdjust = kPAim * headingError - min_aim_command;
    } else if (tx < 1.0) {
      steeringAdjust = kPAim * headingError + min_aim_command;
    }

    return steeringAdjust;
  }

  /**
   * Adjusts distance based on the vertical offset
   * 
   * @return the value to adjust distance by to get it within range of the target
   */
  public double adjustDistance() {
    double distanceError = (double) -ty;
    double distanceAdjust = kPDistance * distanceError;

    return distanceAdjust;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
