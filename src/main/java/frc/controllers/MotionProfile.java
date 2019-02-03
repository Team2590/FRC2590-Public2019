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
public class MotionProfile implements Controller {

    private double kP;
    private double kI;
    private double kV;
    private double kA;
    private double maxVel;
    private double maxAcc;

    // These objects allow us to read from and write
    // to the sensors and motor, respectively
    private PIDSource source;
    private PIDOutput output;

    // start and end points of the profile
    private double tolerance;

    // termination of the controller
    private boolean done;

    // checks whether velocity profile is trapezoidal or triangular
    private boolean isTrapezoidal;

    // cruising section of profile
    private double travelDistance;
    private double cappedMaxVel;

    // acceleration distance (ends of the profile)
    private double accelerationDistance;

    // iteration count and breakpoints for acceleration changes in profile
    private int count;
    private int i1; // iteration step from accel to cruising
    private int i2; // iteration step from cruising to decel

    // error sum for an integral feedback
    private double errorSum;

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
    public MotionProfile(double kP, double kI, double kV, double kA, double maxVel, double maxAcc, double tolerance, PIDSource source,
            PIDOutput output) {
        this.kP = kP;
        this.kI = kI;
        this.kV = kV;
        this.kA = kA;
        this.maxVel = maxVel;
        this.maxAcc = maxAcc;
        this.source = source;
        this.output = output;
        this.tolerance = tolerance;

        source.setPIDSourceType(PIDSourceType.kDisplacement);

        done = true;

        isTrapezoidal = false;

        travelDistance = 0.0;
        cappedMaxVel = maxVel;

        accelerationDistance = 0.0;

        count = 0;
        i1 = 0;
        i2 = 0;

        errorSum = 0.0;

        backwards = false;
    }

    public void setSetpoint(double setpoint) {
        // gets the current position of the source for feedback control
        double currentPos = getSourceDistance();

        // calculates path direction and distance
        backwards = (currentPos > setpoint);
        travelDistance = Math.abs(setpoint - currentPos);

        // Determines the fastest velocity the system will travel
        // The capped velocity is the minimum between the desired max velocity and the
        // fastest velocity achievable by half the distance profile
        // Essentially, the max vel is capped by the system's capabilities
        cappedMaxVel = Math.min(maxVel, Math.sqrt(maxAcc * travelDistance));
        accelerationDistance = Math.pow(cappedMaxVel, 2) / (2 * maxAcc); // d = v^2 / 2a

        // sets the first iteration step, aka when the profile switches from accel to
        // cruising
        i1 = (int) (cappedMaxVel / (maxAcc * dt));

        // creates a trapezoidal path if the robot isn't accelerating for the entire
        isTrapezoidal = (accelerationDistance < (travelDistance / 2));
        if (isTrapezoidal) {
            i2 = (int) (i1 + (travelDistance - 2 * accelerationDistance) / (cappedMaxVel * dt));
        } else {
            // the velocity profile is triangular, there is no period of constant velocity
            accelerationDistance = 0.5 * travelDistance;
            i2 = i1;
        }

        count = 0;
        errorSum = 0.0;
    }

    public void calculate() {
        if (!done) {
            double currentPos = getSourceDistance();
            double output_position = 0.0;
            double output_velocity = 0.0;
            double output_acceleration = 0.0;

            count++; // counter for the number of iterated timesteps

            if (count < i1) { // accelerating
                output_acceleration = maxAcc;
                output_velocity = maxAcc * count * dt;
                output_position = 0.5 * output_velocity * count * dt; // = 0.5at^2

            } else if (count >= i1 && count < i2) { // cruising
                output_acceleration = 0;
                output_velocity = cappedMaxVel;
                output_position = cappedMaxVel * (count - i1) * dt;

            } else if (count > i2) { // decelerating
                output_acceleration = -maxAcc;
                output_velocity = cappedMaxVel - maxAcc * (count - i2) * dt;
                output_position = (travelDistance - accelerationDistance) + output_velocity * (count - i2) * dt
                        + 0.5 * output_acceleration * Math.pow((count - i2), 2) * Math.pow(dt, 2);
                // the above code is essentially df = di + vt + 0.5at^2
            }

            double error = output_position - currentPos;

            //profile is finished, output 0.0 to motors and exit
            if(Math.abs(error) < tolerance) {
                done = true;
                output.pidWrite(0.0);
            }
            errorSum += error * dt;

            //kV and kA are feedforward, kP and kI are feedback 
            output.pidWrite(kV * output_velocity + kA * output_acceleration + kP * error + kI * errorSum);
        }
        output.pidWrite(0.0);
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
