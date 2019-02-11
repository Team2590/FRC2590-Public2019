/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Camera object for Retroreflective vision or general driving
 */
public class Camera extends Subsystem {
  
  //singleton
  private static Camera cameraInstance = null;
  public static Camera getCameraInstance() {
    if(cameraInstance == null) {
      cameraInstance = new Camera();
    }
    return cameraInstance;
  }

  //Camera Types (axis, usb cam, limelight)
  //access the LimeLight web pipeline via: http://10.te.am.11:5801
    
  public Camera() {

  }

  public void setCameraIp(String ip) {

  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
