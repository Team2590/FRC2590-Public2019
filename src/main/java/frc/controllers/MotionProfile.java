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

/**
 * Add your docs here.
 */
public class MotionProfile implements Controller {

    private double kP;
    private double kV;
    private double kA;
    private double maxVel;
    private double maxAcc;

    private PIDSource source;
    private PIDOutput output;

    // start and end points of the profile
    private double startPoint;
    private double endPoint;

    // motor outputs and termination of the controller
    private boolean done;
    private double[] desiredOutputs;

    // checks whether velocity profile is trapezoidal or triangular
    private boolean isTrapezoidal;

    // cruising section of profile
    private double travelDistance;
    private double cruisingDistance;
    private double adjustedMaxVel;

    // acceleration distance (ends of the profile)
    private double accelerationDistance;

    // whether the profile is read forward or backwards
    private boolean backwards;

    // future values
    private double nextVelocity;

    /**
     * Motion Profile Controller Allows for smooth and accurate path following
     * 
     * @param kP: proportional feedback
     * @param kV: velocity feedforward
     * @param kA: acceleration feedforward
     * @param maxVel: desired max velocity
     * @param maxAcc: desired max acceleration
     * @param source: sensor source (Encoder, Gyro, Potentiometer, etc)
     * @param output: motor output (TalonSRX, VictorSPX, CANSparkMax, etc)
     */
    public MotionProfile(double kP, double kV, double kA, double maxVel, double maxAcc, PIDSource source,
            PIDOutput output) {
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        this.source = source;
        this.output = output;

        source.setPIDSourceType(PIDSourceType.kDisplacement);

        startPoint = 0.0;
        endPoint = 0.0;

        done = true;
        desiredOutputs = new double[3]; // desired position, desired velocity, desired acceleration

        isTrapezoidal = false;

        travelDistance = 0.0;
        cruisingDistance = 0.0;
        adjustedMaxVel = maxVel;

        accelerationDistance = 0.0;

        backwards = false;

        nextVelocity = 0.0;
    }

    public void setSetpoint(double setpoint) {

        double current = source.pidGet();
        done = false;

        // sets the start and end points of the profile
        startPoint = source.pidGet();
        endPoint = setpoint;

        nextVelocity = 0;

        // resets the output array for each new profile
        desiredOutputs = new double[3];
        desiredOutputs[0] = current; // desired position
        desiredOutputs[1] = 0.0; // desired velocity
        desiredOutputs[2] = 0.0; // desired acceleration

        // calculates path direction and distance
        backwards = (current > setpoint);
        travelDistance = Math.abs(setpoint - current);

        // determines the fastest velocity the system will travel
        // the adjusted velocity is the minimum between the desired max velocity and the
        // fastest velocity achievable by half the distance profile (given a max
        // acceleration)
        adjustedMaxVel = Math.min(maxVel, Math.sqrt(maxAcc * travelDistance));
        accelerationDistance = Math.pow(adjustedMaxVel, 2) / (2 * maxAcc); // d = v^2 / 2a

        // creates a trapezoidal path if the robot isn't accelerating for the entire
        isTrapezoidal = (accelerationDistance < (travelDistance / 2));
        if (isTrapezoidal) {
            cruisingDistance = (travelDistance - 2 * accelerationDistance);
        } else {
            // the velocity profile is triangular, there is no period of constant velocity
            cruisingDistance = 0.0;
        }
    }

    public void calculate() {
        if (!done) {
            double current = source.pidGet();

            // distances from start and end points
            double distanceFromStart = Math.abs(startPoint - current);
            double distanceFromEnd = Math.abs(endPoint - current);



            //checks if the path is finished
            if(distanceFromStart >= travelDistance) {
                setDesiredOutputArray(endPoint, 0.0, 0.0);
                done = true;
                output.pidWrite(0.0);
            }

            //calculates acceleration/deceleration phases of profile
            if(distanceFromStart <= accelerationDistance) { //accel


            } else if(distanceFromStart > (accelerationDistance + cruisingDistance)) { //decel

            }

            //calculates the cruising phase of the profile
            if(isTrapezoidal && (distanceFromStart > accelerationDistance) && (distanceFromEnd > accelerationDistance)) {

            }

            output.pidWrite(desiredOutputs[0] * kP
                            + desiredOutputs[1] * kV
                             + desiredOutputs[2] * kA);

        }

        //controller is done, output 0.0 just in case
        output.pidWrite(0.0);
    }

    private void setDesiredOutputArray(double position, double velocity, double acceleration) {
        int direction = (backwards ? -1 : 1);

        desiredOutputs[0] += position * direction;
        desiredOutputs[1] = velocity * direction;
        desiredOutputs[2] = acceleration * direction;

        nextVelocity = velocity;
    }

    public boolean isDone() {
        return true;
    }
}
