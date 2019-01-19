/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Limelight camera class for vision processing Uses network tables relayed from
 * the camera to track retroreflective targets
 */
public class Limelight extends Subsystem implements LimelightSettings {

  // singleton
  private static Camera cameraInstance = null;

  public static Camera getCameraInstance() {
    if (cameraInstance == null) {
      cameraInstance = new Camera();
    }
    return cameraInstance;
  }

  // network table from limelight camera
  NetworkTable table;

  // target coordinates and values
  double tx, ty, tz;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = 0.0;
    ty = 0.0;
    tz = 0.0;

  }

  /**
   * Calculates the angleto the retro-reflective target
   * 
   * @return the vertical angle from the camera to the target
   */
  public double veticalAngleToTarget() {
    try {
      // difference between target and camera Y values, times the degrees/pixel
      return ((ty - CY) * YDPP);
    } catch (Exception e) {
      DriverStation.reportError("Target Not Found", false);
      return 0;
    }
  }

  public double hAngleToTarget() {
    try {
      // difference between target and camera X values, times the degrees/pixel
      return ((tx - CX) * XDPP);
    } catch (Exception e) {
      DriverStation.reportError("Target Not Found!", false);
      return 0;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}