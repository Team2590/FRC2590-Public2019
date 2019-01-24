/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

/**
 * Bang Bang controller for rudimentary motion control
 * Constantly oscillates a motor between two values to compensate for overshoot
 */
public class BangBang implements Controller {

    private double setpoint;
    private double tolerance;
    private double controlSpeed;

    private boolean done;

    public BangBang(double tolerance, double controlSpeed) {
        this.setpoint = 0;
        this.tolerance = tolerance;
        this.controlSpeed = controlSpeed;

        this.done = true;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double current) {

        //checks if controller is within a reasonable tolerance to stop the motor
        if(Math.abs(current - setpoint) < tolerance) {
            done = true;
            return 0.0; //cuts speed entirely, relies on motor's inertia to keep speed
        }

        if(current > setpoint) {
            return -controlSpeed; //drive the motor in reverse to drop current speed
        } else {
            return controlSpeed; //drive the motor forward to increase current speed
        }
    }

    public boolean isDone() {
        return done;
    }
}
