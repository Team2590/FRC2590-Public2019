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
 * Creates a velocity profile for the system to travel smoothly and accurately
 * 
 * @author Harsh Padhye
 */
public class OldMotionProfile implements Controller {

    private double kP;
    private double kV;
    private double kA;
    private double maxVel;
    private double maxAcc;

    // These objects allow us to read from and write
    // to the sensors and motor, respectively
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
    private double cappedMaxVel;

    // acceleration distance (ends of the profile)
    private double accelerationDistance;

    // whether the profile is read forward or backwards
    private boolean backwards;

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
    public OldMotionProfile(double kP, double kV, double kA, double maxVel, double maxAcc, PIDSource source,
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
        cappedMaxVel = maxVel;

        accelerationDistance = 0.0;

        backwards = false;
    }

    public void setSetpoint(double setpoint) {

        double currentPos = getSourceDistance();

        // sets the start and end points of the profile
        startPoint = currentPos;
        endPoint = setpoint;

        // resets the output array for each new profile
        desiredOutputs = new double[3];
        desiredOutputs[0] = currentPos; // desired position
        desiredOutputs[1] = 0.0; // desired velocity
        desiredOutputs[2] = 0.0; // desired acceleration

        // calculates path direction and distance
        backwards = (currentPos > setpoint);
        travelDistance = Math.abs(setpoint - currentPos);

        // Determines the fastest velocity the system will travel
        // The capped velocity is the minimum between the desired max velocity and the
        // fastest velocity achievable by half the distance profile
        // Essentially, the max vel is capped by the system's capabilities
        cappedMaxVel = Math.min(maxVel, Math.sqrt(maxAcc * travelDistance));
        accelerationDistance = Math.pow(cappedMaxVel, 2) / (2 * maxAcc); // d = v^2 / 2a

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

            // gets the sensor's current values for distance and rate
            // used in calculation for intial position and velocity
            double currentVel = getSourceRate();
            double currentPos = getSourceDistance();

            // distances from start and end points
            double distanceFromStart = Math.abs(startPoint - currentPos);
            double distanceFromEnd = Math.abs(endPoint - currentPos);

            // max and min reach velocities (given the next velocity, what is the
            // fastest/slowest you'll go)
            double maxReachVel = Math.sqrt(Math.pow(currentVel, 2) + 2 * maxAcc * distanceFromEnd);
            double minReachVel = Math.sqrt(Math.pow(currentVel, 2) - 2 * maxAcc * distanceFromEnd);

            // makes sure the velocity is non-negative
            if (!backwards) { // path read forwards
                if (minReachVel < 0 || currentVel < 0) {
                    currentVel = Math.min(cappedMaxVel, maxReachVel);
                }
            } else { // path read in reverse
                if (minReachVel < 0) {
                    currentVel = Math.min(cappedMaxVel, maxReachVel);
                }
            }

            // checks if the path is finished
            if (distanceFromStart >= travelDistance) {
                setDesiredOutputArray(endPoint, 0.0, 0.0);
                done = true;
                output.pidWrite(0.0);
            }

            // calculates acceleration/deceleration phases of profile
            if (distanceFromStart <= accelerationDistance) { // accel
                setDesiredOutputArray(currentVel * dt + 0.5 * maxAcc * (dt * dt), currentVel + maxAcc * dt, maxAcc);

            } else if (distanceFromStart > (accelerationDistance + cruisingDistance)) { // decel
                // kinematics subtract the acceleration term because it is negative
                setDesiredOutputArray(currentVel * dt - 0.5 * maxAcc * (dt * dt), currentVel - maxAcc * dt, -maxAcc);

            }

            // calculates the cruising phase of the profile
            if (isTrapezoidal && (distanceFromStart > accelerationDistance)
                    && (distanceFromEnd > accelerationDistance)) {
                // drive at a constant speed (acceleration = 0)
                setDesiredOutputArray(currentVel * dt, currentVel, 0);
            }

            output.pidWrite(desiredOutputs[0] * kP + desiredOutputs[1] * kV + desiredOutputs[2] * kA);

        }

        // controller is done, output 0.0 just in case
        output.pidWrite(0.0);
    }

    /**
     * Sets the output array, a holster for the desired pos, vel, and acc for the
     * next iteration of the profile.
     * 
     * @param position     change in position based on profile
     * @param velocity     next velocity in profile
     * @param acceleration next acceleration in profile
     */
    private void setDesiredOutputArray(double position, double velocity, double acceleration) {
        // multiplies all output vars by -1 if reading the traj backwards
        int direction = (backwards ? -1 : 1);

        desiredOutputs[0] = position * direction; // kinematics calculate delta displacement, automatic error calc
        desiredOutputs[1] = velocity * direction;
        desiredOutputs[2] = acceleration * direction;
    }

    public boolean isDone() {
        return true;
    }

    public double getSourceDistance() {
        source.setPIDSourceType(PIDSourceType.kDisplacement);
        return source.pidGet();
    }

    public double getSourceRate() {
        source.setPIDSourceType(PIDSourceType.kRate);
        return source.pidGet();
    }
}
