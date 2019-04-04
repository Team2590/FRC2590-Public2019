package frc.controllers;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class ConstantCurrent implements Controller {

  private double command;
  private double positionLimit;
  private boolean isLimiting;
  private PIDSource source;
  private PIDOutput output;

  public ConstantCurrent (PIDSource source, PIDOutput output) {
    this.source = source;
    this.output = output;
    isLimiting = false;
  }

  @Override
  public void setSetpoint(double setpoint) {
    command = setpoint;
  }

  @Override
  public void calculate() {
    if(isLimiting && source.pidGet() > positionLimit) {
      output.pidWrite(0.0);
    }
    output.pidWrite(command);
  }

  /**
   * Sets a limit to how far the controller will run
   * Can prevent stalling and excess torque
   * @param positionLimit farthest you want the controller to run
   */
  public void setPositionLimit(double positionLimit) {
    this.positionLimit = positionLimit;
    isLimiting = true;
  }

  @Override
  public boolean isDone() {
    return true;
  }
}
