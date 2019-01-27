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
 * Bang Bang controller for basic motion control and tracking
 * Constantly oscillates a motor between two values to compensate for overshoot
 * @author Harsh Padhye
 */
public class BangBang implements Controller {

    private double setpoint;
    private double tolerance;
    private double controlSpeed;

    private boolean done;

    private PIDSource source;
    private PIDOutput output;

    public BangBang(double tolerance, double controlSpeed, PIDSource source, PIDOutput output) {
        this.setpoint = 0;
        this.tolerance = tolerance;
        this.controlSpeed = controlSpeed;

        this.source = source;
        this.output = output;

        this.done = true;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void calculate() {

        //checks if controller is within a reasonable tolerance to stop the motor
        if(Math.abs(source.pidGet() - setpoint) < tolerance) {
            done = true;
            output.pidWrite(0.0); //cuts speed entirely, relies on motor's inertia to keep speed
        }

        if(source.pidGet() > setpoint) {
            output.pidWrite(-controlSpeed); //drive the motor in reverse to drop current speed
        } else {
            output.pidWrite(controlSpeed); //drive the motor forward to increase current speed
        }
    }

    public boolean isDone() {
        return done;
    }
}
