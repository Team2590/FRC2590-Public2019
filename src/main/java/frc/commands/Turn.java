/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import frc.auto.NemesisRunnable;
import frc.robot.Robot;

/**
 * Turns to a given angle
 * @author Chinmay Savanur
 * */
public class Turn {

  private double setpoint;
  private boolean hasStarted;

  public Turn(double angle) {
    setpoint = angle;
    hasStarted = false;
    System.out.println("stp " + setpoint);
  }

  @Override
  public void run() {
    if(!hasStarted) {
      
    }
  }

  @Override
  public boolean isDone() {
      return true;
  }

  @Override 
  public String getKey() {
      return null;
  }

}
