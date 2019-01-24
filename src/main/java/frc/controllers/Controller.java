/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

/**
 * Basic template for a feedback controller
 */
public interface Controller {

  /**
   * resets the desired setpoint of the controller
   * @param stp: setpoint
   */
  public void setSetpoint(double setpoint);

  /**
   * calculates the output power of the controller based on the current position
   * @param current: current value of sensor
   * @return the output power to the motor
   */
  public double calculate(double current);

  /**
   * checks whether the current value is within the required threshold to stop the controller
   * @return whether the controller has finished its feedback loop
   */
  public boolean isDone();
}