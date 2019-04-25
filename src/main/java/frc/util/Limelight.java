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
  private double tx, ty, tv, ts;

  private double[] camtran;
  private double[] defaultArray = { 0, 0, 0, 0, 0, 0 };

  private boolean limelightOn;

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table.getEntry("pipeline").setNumber(3);

    tx = 0.0; // Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    ty = 0.0; // Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
    tv = 0.0; // Whether the limelight has any valid targets (0 or 1)
    ts = 0.0; // Skew or rotation (-90 degrees to 0 degrees)

    camtran = new double[6];

    limelightOn = false;
  }

  public void update() {
    tx = table.getEntry("tx").getDouble(0);
    ty = table.getEntry("ty").getDouble(0);
    tv = table.getEntry("tv").getDouble(0);
    ts = table.getEntry("ts").getDouble(0);
    camtran = table.getEntry("camtran").getDoubleArray(defaultArray);
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
  public double getHorizontalAngleToTarget() {
    if (tv == 1) {
      return tx;
    } else {
      return 0.0;
    }
  }

  public double getSkew() {
    if (tv == 1) {
      return tx;
    } else {
      return 0.0;
    }
  }

  public double getCamTranZ() {
    if (tv == 1) {
      return camtran[CAMTRAN_Z];
    } else {
      return 0.0;
    }
  }

  public double getCamTranX() {
    if (tv == 1) {
      return camtran[CAMTRAN_X];
    } else {
      return 0.0;
    }
  }

  public double getCamTranYaw() {
    if (tv == 1) {
      return camtran[CAMTRAN_YAW];
    } else {
      return 0.0;
    }
  }

  public boolean has3DLoc() {
    return (Math.abs(camtran[CAMTRAN_YAW]) > 0.0001);
  }

  public boolean hasTarget() {
    return (tv == 1);
  }

  /**
   * Calculates angle to become perpendicular to target
   * 
   * @return the horizontal angle between the target perpendicular and the
   *         robot-target line
   */
  public double getAngle2() {
    return getCamTranYaw() - getHorizontalAngleToTarget();
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
