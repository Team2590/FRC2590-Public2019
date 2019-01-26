/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Timer;

/**
 * Basic Proportional, Integral, Derivative feedback control loop
 */
public class PID implements Controller {

    double kP, kI, kD; // control gains (proportional, integral, derivative)
    double setpoint;
    double errorSum;
    double lastError;
    double lastTime;
    double tolerance;

    int cycles;

    boolean done;

    PIDSource source;
    PIDOutput output;

    // consider using PID Source and PIDOutput so that calculate can be written
    // directly from this class rather than returning a function
    public PID(double kP, double kI, double kD, double tolerance, PIDSource source, PIDOutput output) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.tolerance = tolerance;

        this.source = source;
        this.output = output;

        errorSum = 0.0;
        lastError = 0.0;
        lastTime = 0.0;
        cycles = 0;

        done = true;
    }

    public void setSetpoint(double setpoint) {
        done = false;
        errorSum = 0.0;
        this.setpoint = setpoint;
    }

    public void calculate() {
        double error = setpoint - source.pidGet(); // difference between desired value and current value
        double time = Timer.getFPGATimestamp() * 1000; // current time
        double dt = time - lastTime; // timestep
        double errorDelta = (error - lastError); // change in error

        errorSum += error * dt; // integrates error over time

        // sets the last values for the next iteration
        lastTime = time;
        lastError = error;

        if (Math.abs(error) < tolerance) {
            cycles++;
        } else {
            cycles = 0;
        }

        done = (cycles > 5); // checks 5 times whether the robot is on target

        if (done) {
            output.pidWrite(0.0);
        }

        output.pidWrite((kP * error) + (kI * errorSum) + (kD * errorDelta));
    }

    public boolean isDone() {
        return done;
    }

}
