/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Basic Proportional, Integral, Derivative feedback control loop
 * 
 * @author Harsh Padhye
 */
public class PID implements Controller {

    private double kP, kI, kD, kH, kR; // control gains (proportional, integral, derivative, heading, turn rate)
    private double setpoint_pos;
    private double setpoint_angle;
    private double setpoint_angleRate;
    private double errorSum;
    private double lastError;
    private double tolerance;

    private int cycles;

    private boolean done;

    private PIDSource source;
    private PIDOutput output;
    private Gyro gyro;

    /**
     * Proportional, integral, and derivative feedback controller
     * 
     * @param kP        Proportional gain
     * @param kI        Integral gain
     * @param kD        Derivative gain
     * @param tolerance Acceptable range from setpoint to stop the controller
     * @param source    Sensor source (Encoder, Gyro, Potentiometer, etc)
     * @param output    Motor output (TalonSRX, VictorSPX, CANSparkMax, etc)
     */
    public PID(double kP, double kI, double kD, double tolerance, PIDSource source, PIDOutput output) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.tolerance = tolerance;

        this.source = source;
        this.output = output;

        source.setPIDSourceType(PIDSourceType.kDisplacement);

        errorSum = 0.0;
        lastError = 0.0;
        cycles = 0;

        done = true;
    }

    /**
     * Proportional, integral, and derivative feedback controller Allows the input
     * of a second sensor (gyro) for heading correction
     * 
     * @param kP        Proportional gain
     * @param kI        Integral gain
     * @param kD        Derivative gain
     * @param kH        Heading gain
     * @param kR        Angular rate gain
     * @param tolerance Acceptable range from setpoint to stop the controller
     * @param source    Sensor source (Encoder, Gyro, Potentiometer, etc)
     * @param output    Motor output (TalonSRX, VictorSPX, CANSparkMax, etc)
     * @param gyro      Gyro sensor for current angle and angular rate
     */
    public PID(double kP, double kI, double kD, double kH, double kR, double tolerance, PIDSource source,
            PIDOutput output, Gyro gyro) {

        this(kP, kI, kD, tolerance, source, output);

        this.kH = kH;
        this.kR = kR;
        this.gyro = gyro;
    }

    /**
     * Sets the desired position, angle, and turn rate of the controller
     * 
     * @param setpoint_pos       desired position
     * @param setpoint_angle     desired angle
     * @param setpoint_angleRate desired turn rate
     */
    public void setSetpoint(double setpoint_pos, double setpoint_angle, double setpoint_angleRate) {
        done = false;
        errorSum = 0.0;
        this.setpoint_pos = setpoint_pos;
        this.setpoint_angle = setpoint_angle;
        this.setpoint_angleRate = setpoint_angleRate;
    }

    /**
     * Sets the desired position of the controller For use without simultaneous gyro
     * integration
     * 
     * @param setpoint_pos desired position
     */
    public void setSetpoint(double setpoint_pos) {
        this.setSetpoint(setpoint_pos, 0.0, 0.0);
    }

    public void calculate() {
        double error = setpoint_pos - source.pidGet(); // difference between desired value and current value
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
        } else {
            output.pidWrite((kP * error) + (kI * errorSum) + (kD * errorDelta) + headingCompensation()
                    + turnRateCompensation());
        }
    }

    /**
     * Calculates the appropriate motor power to compensate for angle error
     * 
     * @return motor power to compensate for angle error
     */
    private double headingCompensation() {
        if (gyro == null) {
            return 0.0;
        }
        return kH * (setpoint_angle - gyro.getAngle());
    }

    /**
     * Calculates the dampening factor for turn rate
     * 
     * @return motor power to compensate for turn rate error
     */
    private double turnRateCompensation() {
        if (gyro == null) {
            return 0.0;
        }
        return kR * (setpoint_angleRate - gyro.getRate());
    }

    public boolean isDone() {
        return done;
    }

}
