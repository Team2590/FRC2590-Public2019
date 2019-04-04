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
 * @author Harsh Padhye, Chinmay Savanur
 * 
 *         Access the Limelight web pipeline via: http://10.25.90.11:5801 Access
 *         the Limelight camera feed via: http://10.25.90.11:5800 Access the
 *         Limelight IP camera in GRIP via: http://10.25.90.11:5802
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
  private NetworkTable table;

  // target coordinates and values
  private double tx, ty, tz, tv, tl;

  private boolean limelightOn;

  private boolean isInDelay;
  private int delayCounter;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(3);

    tx = 0.0; // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    ty = 0.0; // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    tz = 0.0; // distance from the camera to the target
    tv = 0.0; // whether the limelight has any valid targets (0 or 1)
    tl = 0.0;

    limelightOn = false;
    isInDelay = false;
    delayCounter = 0;
  }

  public void update() {
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    tv = table.getEntry("tv").getDouble(0);
    tl = table.getEntry("tl").getDouble(0);
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

  public boolean isLimelightOn() {
    return limelightOn;
  }

  public void turnLimelightOn() {
    table.getEntry("ledMode").setNumber(3);
    limelightOn = true;
  }

  public void turnLimelightOff() {
    table.getEntry("ledMode").setNumber(1);
    limelightOn = false;
  }

  public boolean getDelayState() {
    return isInDelay;
  }

  public int getDelayCount() {
    return delayCounter;
  }

  public void startDelay() {
    delayCounter = 0;
    isInDelay = true;
  }

  public void stopDelay() {
    delayCounter = 0;
    isInDelay = false;
  }

  public void incrementCounter() {
    delayCounter++;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
