/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

/**
 * Add your docs here.
 */
public class PID implements Controller {

    double kP, kI, kD; //control gains (proportional, integral, derivative)
    double setpoint;
    double errorSum;
    boolean done;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        errorSum = 0.0;

        done = true;
    }

    public void setSetpoint(double setpoint) {
        done = false;
        this.setpoint = setpoint;
    }

    public double calculate(double current) {
        return 0.0;
    }

    public boolean isDone() {
        return done;
    }
}
