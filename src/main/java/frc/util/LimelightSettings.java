/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

/**
 * Add your docs here.
 */
public interface LimelightSettings {

    // image size in pixels
    public static final int IMAGE_WIDTH = 320;
    public static final int IMAGE_HEIGHT = 240;

    // field of view for limelight cam
    public static final double XFIELD = 0.0;
    public static final double YFIELD = 0.0;

    // center pixels
    public static final double CX = IMAGE_WIDTH / 2.0;
    public static final double CY = IMAGE_HEIGHT / 2.0;

    // degrees per pixel
    public static final double XDPP = XFIELD / IMAGE_WIDTH;
    public static final double YDPP = YFIELD / IMAGE_HEIGHT;

    // dependent on the mounting of the camera, field measurements, etc.
    public static final double CAMERA_ANGLE = 0; // in degrees
    public static final double TARGET_HEIGHT = 0; // in inches
    public static final double CAMERA_HEIGHT = 4; //in inches

}
