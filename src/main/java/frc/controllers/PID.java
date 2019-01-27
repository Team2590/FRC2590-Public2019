/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * Basic Proportional, Integral, Derivative feedback control loop
 * @author Harsh Padhye
 */
public class PID implements Controller {

    private double kP, kI, kD; // control gains (proportional, integral, derivative)
    private double setpoint;
    private double errorSum;
    private double lastError;
    private double tolerance;

    private int cycles;

    private boolean done;

    private PIDSource source;
    private PIDOutput output;

    /**
     * Proportional, integral, and derivative feedback controller
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param tolerance Acceptable range from setpoint to stop the controller
     * @param source Sensor source (Encoder, Gyro, Potentiometer, etc)
     * @param output Motor output (TalonSRX, VictorSPX, CANSparkMax, etc)
     */
    public PID(double kP, double kI, double kD, double tolerance, PIDSource source, PIDOutput output) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.tolerance = tolerance;

        this.source = source;
        this.output = output;

        errorSum = 0.0;
        lastError = 0.0;
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
        double errorDelta = (error - lastError); // change in error

        errorSum += error * dt; // integrates error over time

        // sets the last values for the next iteration
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
