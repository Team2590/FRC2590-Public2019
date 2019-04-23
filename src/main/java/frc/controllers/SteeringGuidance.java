/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class SteeringGuidance implements Controller {

    private double kP, kI, kD;

    private double drivetrainConstant;

    private Encoder[] encoders;

    private double z, x, yaw, horizontalOffset;

    private int count;
    private double rInv;

    private double errorSum, lastError;

    private double lastYaw, lastZ, lastX;

    public SteeringGuidance(double kP, double kI, double kD, double drivetrainConstant, Encoder[] encoders) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.drivetrainConstant = drivetrainConstant;

        this.encoders = encoders;

        init();
    }

    public void init() {
        System.out.println("initing");

        z = 0.0;
        x = 0.0;
        yaw = 0.0;
        horizontalOffset = 0.0;

        count = 0;
        rInv = 0.0;

        errorSum = 0.0;
        lastError = 0.0;

        lastYaw = 0.0;
        lastZ = 0.0;
        lastX = 0.0;
    }

    public double calculateOutput(double horizontalOffset, double yaw, double z, double x) {
        this.horizontalOffset = horizontalOffset;
        this.yaw = yaw;
        this.z = z;
        this.x = x;

        double turnRate = 0.0;
        double velocity = getEncodersAverageRate();

        if (Math.abs(yaw) > 0.0001) {
            double distanceSquared = distanceSquared();
            double rInvCurrent = 2 * x / distanceSquared;

            count++;

            rInv = ((count - 1) / (count)) * rInv + (rInvCurrent / count);

            turnRate = kP * (-yaw + 2 * horizontalOffset) + rInv * velocity;
        } else {
            turnRate = rInv * velocity;
        }

        System.out.println("horizontalOffset " + horizontalOffset + " yaw " + yaw + " z " + z + " x " + x
                + " turn rate " + turnRate + " velocity " + velocity);

        return (Double.isNaN(drivetrainConstant * turnRate) ? 0.0 : drivetrainConstant * turnRate);
    }

    /**
     * 
     * 
     * @param horizontalOffset the horizontal angle (in degrees) from the camera to
     *                         the target
     * @param yaw              how far the robot should turn to become parallel to
     *                         the line going into the target / perpendicular to the
     *                         target
     * @param z                the distance (in inches) from the camera to the
     *                         target in the x-direction
     * @param x                the distance (in inches) from the camera to the
     *                         target in the y-direction
     * @return the turn power required to correct for the error and get
     *         perpendicular and in line with the retroflective tape
     */
    public double calculate(double horizontalOffset, double yaw, double z, double x) {
        double velocity = getEncodersAverageRate();

        this.horizontalOffset = horizontalOffset;

        this.yaw = (Math.abs(yaw) < 0.0001 ? estimateYaw(velocity) : yaw);
        this.z = (Math.abs(z) < 0.0001 ? estimateZ(velocity) : z);
        this.x = (Math.abs(x) < 0.0001 ? estimateX(velocity) : x);

        double error = -this.yaw + 2 * this.horizontalOffset;

        lastYaw = this.yaw;
        lastZ = this.z;
        lastX = this.x;

        System.out.println("horizontalOffset " + horizontalOffset + "this.horizontalOffset " + this.horizontalOffset
                + " yaw " + yaw + " this.yaw " + this.yaw + " z " + z + " this.z " + this.z + " x " + x + " this.x "
                + this.x + " retVal " + kP * error);

        // in case kP * error is not a number, sets motor power to 0.0 so that robot
        // does not spin in circles
        return (Double.isNaN(kP * error) ? 0.0 : kP * error);
    }

    /**
     * Calculates what the value of yaw should be at a given time based on previous
     * values and the current velocity. This method is used when the limelight
     * cannot calculate 3D Localization
     * 
     * @param velocity the current velocity that the robot is traveling when the
     *                 limelight loses values from 3D Localization
     * @return an estimated value of yaw given velocity and the previous values of x
     *         and yaw
     */
    private double estimateYaw(double velocity) {
        double distanceSquared = getDistanceSquared();
        double previousX = lastX;
        double previousYaw = lastYaw;

        return previousYaw + Math.toDegrees(dt * (velocity / distanceSquared) * 2 * previousX);
    }

    /**
     * Calculates what the value of z should be at a given time based on previous
     * values and the current velocity. This method is used when the limelight
     * cannot calculate 3D Localization
     * 
     * @param velocity the current velocity that the robot is traveling when the
     *                 limelight loses values from 3D Localization
     * @return an estimated value of z given velocity and the previous values of z
     *         and yaw
     */
    private double estimateZ(double velocity) {
        double previousZ = lastZ;
        double previousYaw = lastYaw;

        return previousZ + dt * velocity * Math.cos(previousYaw);
    }

    /**
     * Calculates what the value of x should be at a given time based on previous
     * values and the current velocity. This method is used when the limelight
     * cannot calculate 3D Localization
     * 
     * @param velocity the current velocity that the robot is traveling when the
     *                 limelight loses values from 3D Localization
     * @return an estimated value of x given velocity and the previous values of x
     *         and yaw
     */
    private double estimateX(double velocity) {
        double previousX = lastX;
        double previousYaw = lastYaw;

        return previousX + dt * velocity * Math.sin(previousYaw);
    }

    /**
     * Calculates the distance squared using the last known values of z and x from
     * limelight
     * 
     * @return distance squared from limelight
     */
    private double getDistanceSquared() {
        return Math.pow(lastZ, 2) + Math.pow(lastX, 2);
    }

    private double distanceSquared() {
        return Math.pow(z, 2) + Math.pow(x, 2);
    }

    /**
     * Returns average of current rates of drive encoders
     * 
     * @return average of encoders' current rate
     */
    private double getEncodersAverageRate() {
        Encoder leftDriveEncoder = encoders[0];
        Encoder rightDriveEncoder = encoders[1];

        return (leftDriveEncoder.getRate() + rightDriveEncoder.getRate()) / 2.0;
    }

    /**
     * Consistency with other controller classes
     */

    @Override
    public void setSetpoint(double setpoint) {

    }

    @Override
    public void calculate() {

    }

    @Override
    public boolean isDone() {
        return false;
    }

}
