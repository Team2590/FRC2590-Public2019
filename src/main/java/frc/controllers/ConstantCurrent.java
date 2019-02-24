package frc.controllers;

import edu.wpi.first.wpilibj.PIDOutput;

public class ConstantCurrent implements Controller {

  private double volts;
  private PIDOutput output;

  public ConstantCurrent (PIDOutput output) {
    this.output = output;
  }

  @Override
  public void setSetpoint(double setpoint) {
    volts = setpoint;
  }

  @Override
  public void calculate() {
    output.pidWrite(volts);
  }

  @Override
  public boolean isDone() {
    return true;
  }
}
