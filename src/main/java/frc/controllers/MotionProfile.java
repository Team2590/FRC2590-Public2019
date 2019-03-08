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
 * @author Harsh Padhye, Dr. Mr. Jeff Keller, PhD.
 */
public class MotionProfile implements Controller {

    // Profile gains
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

    // start and end points and tolerance of the profile
    private double endPoint;
    private double startPoint;
    private double tolerance;

    // termination of the controller
    private boolean done;

    // adjusted max velocity of profile
    private double capMaxVel;

    // distances of profile and segments
    private double travelDistance;
    private double accelDistance;
    private double cruiseDistance;

    // iteration count and breakpoints for acceleration changes in profile
    private int count;
    private int i1; // iteration step from accel to cruising
    private int i2; // iteration step from cruising to decel

    // error sum for an integral feedback
    private double errorSum;

    // estimated total time to follow the profile
    private double totalTime;

    // direction of the path (1 forwards, -1 backwards)
    private int dir;

    /**
     * Motion Profile Controller allows for smooth and accurate path following
     * 
     * @param kP        proportional feedback
     * @param kI        integral feedback
     * @param kV        velocity feedforward
     * @param kA        acceleration feedforward
     * @param maxVel    desired max velocity
     * @param maxAcc    desired max acceleration
     * @param tolerance acceptable range to stop controller
     * @param source    sensor source (Encoder, Gyro, Potentiometer, etc)
     * @param output    motor output (TalonSRX, VictorSPX, CANSparkMax, etc)
     */
    public MotionProfile(double kP, double kI, double kV, double kA, double maxVel, double maxAcc, double tolerance,
            PIDSource source, PIDOutput output) {
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

        endPoint = 0.0;
        startPoint = 0.0;
        travelDistance = 0.0;
        capMaxVel = maxVel;

        accelDistance = 0.0;
        cruiseDistance = 0.0;

        count = 0;
        i1 = 0;
        i2 = 0;

        errorSum = 0.0;

        totalTime = 0.0;

        dir = 1;
    }

    /**
     * Sets the desired position of the profile
     * 
     * @param setpoint The desired position of the profile
     */
    public void setSetpoint(double setpoint) {
        // gets the current position of the source for feedback control
        startPoint = getSourceDistance();

        // sets the endpoint as the desired setpoint
        endPoint = setpoint;

        // calculates path direction and distance
        dir = (startPoint > setpoint) ? -1 : 1;
        travelDistance = Math.abs(setpoint - startPoint);

        // Determines the fastest velocity the system will travel
        // Essentially, the max vel is capped by the system's capabilities
        capMaxVel = Math.min(maxVel, Math.sqrt(maxAcc * travelDistance));

        // calculates the time required to follow the profile
        totalTime = (travelDistance / capMaxVel) + (capMaxVel / maxAcc);

        // calcualtes the accel and cruise distances
        accelDistance = (capMaxVel * capMaxVel) / (2 * maxAcc); // d = v^2 / 2a
        cruiseDistance = travelDistance - 2 * accelDistance;
        if (cruiseDistance < 0) {
            cruiseDistance = 0; // cruise dist should never be < 0
        }

        // sets the indices, aka when the profile switches from accel to cruise and
        // cruise to decel, respectively
        i1 = (int) (capMaxVel / (maxAcc * dt));
        i2 = (int) (travelDistance / (capMaxVel * dt));

        count = 0; // reset loop counter
        errorSum = 0.0; // reset integrator

        // allows calculate to run
        done = false;
    }

    /**
     * Calculates the outputs of the profile and writes to the motor automatically
     */
    public void calculate() {
        if (!done) {
            double currentPos = getSourceDistance();
            double output_position = 0.0;
            double output_velocity = 0.0;
            double output_acceleration = 0.0;

            count++; // counter for the number of iterated timesteps

            // multiplying by "dir" accounts for the direction of the profile
            if (count < i1) { // accelerating
                output_acceleration = maxAcc * dir;
                output_velocity = output_acceleration * count * dt;
                output_position = startPoint + 0.5 * output_velocity * count * dt; // = 0.5at^2

            } else if (count >= i1 && count <= i2) { // cruising
                output_acceleration = 0;
                output_velocity = capMaxVel * dir;
                output_position = startPoint + (accelDistance * dir) + output_velocity * (count - i1) * dt;

            } else if (count > i2) { // decelerating
                output_acceleration = -maxAcc * dir;
                output_velocity = (capMaxVel - maxAcc * (count - i2) * dt) * dir;
                output_position = startPoint + (travelDistance - accelDistance) * dir
                        + 0.5 * (capMaxVel * capMaxVel - output_velocity * output_velocity) / maxAcc * dir;
            }

            double error = output_position - currentPos;
            errorSum += error * dt;

            // kV and kA are feedforward, kP and kI are feedback
            double command = kV * output_velocity + kA * output_acceleration + kP * error + kI * errorSum;

            // profile is finished, output 0.0 to motors and exit
            if (Math.abs(endPoint - currentPos) < tolerance) {
                done = true;
                command = 0.0;
            }

           System.out.println(
                    count * dt + " " + output_position + " " + currentPos + " " + output_velocity + " " + command);
            output.pidWrite(command);
        }
    }

    /**
     * @return Whether or not the profile has finished
     */
    public boolean isDone() {
        return done;
    }

    /**
     * Ends the controller and resets the iteration steps
     */
    public void endProfile() {
        count = 0;
        i1 = 0;
        i2 = 0;
        done = true;
    }

    /**
     * Sets the source to displacement mode and returns the current position
     * 
     * @return The sensor's current position
     */
    public double getSourceDistance() {
        source.setPIDSourceType(PIDSourceType.kDisplacement);
        return source.pidGet();
    }

    /**
     * Sets the source to rate mode and returns the current rate
     * 
     * @return The sensor's current rate (d/dt of position)
     */
    public double getSourceRate() {
        source.setPIDSourceType(PIDSourceType.kRate);
        return source.pidGet();
    }

    /**
     * @return The estimated time to follow the profile
     */
    public double getProfileTime() {
        return totalTime;
    }
}
