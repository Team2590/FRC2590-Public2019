/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.controllers;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class SteeringGuidance implements Controller {

    private double kP, kI, kD;

    private double drivetrainConstant;

    private PIDSource[] sources;

    private double z, x, yaw, horizontalOffset;

    private int count;
    private double rInv;

    private double errorSum, lastError;

    private double lastZ, lastX, lastYaw;

    public SteeringGuidance(double kP, double kI, double kD, double drivetrainConstant, PIDSource[] sources) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.drivetrainConstant = drivetrainConstant;

        this.sources = sources;

        init();
    }

    public void init() {
        z = 0.0;
        x = 0.0;
        yaw = 0.0;
        horizontalOffset = 0.0;

        count = 0;
        rInv = 0.0;

        errorSum = 0.0;
        lastError = 0.0;
    }

    public double calculateOutput(double z, double x, double yaw, double horizontalOffset) {
        this.z = z;
        this.x = x;
        this.yaw = yaw;
        this.horizontalOffset = horizontalOffset;

        double turnRate = 0.0;
        double velocity = getSourcesAverageRate();

        if (Math.abs(yaw) > 0.0001) {
            double distanceSquared = getDistanceSquared();
            double rInvCurrent = 2 * x / distanceSquared;

            count++;

            rInv = ((count - 1) / (count)) * rInv + (rInvCurrent / count);

            turnRate = kP * (-yaw + 2 * horizontalOffset) + rInv * velocity;
        } else {
            turnRate = rInv * velocity;
        }

        return drivetrainConstant * turnRate;
    }

    public double calculate(double horizontalOffset, double yaw, double z, double x) {
        double velocity = getSourcesAverageRate();

        this.horizontalOffset = horizontalOffset;

        this.yaw = (Math.abs(yaw) < 0.0001 ? estimateYaw(velocity) : yaw);
        this.z = (Math.abs(z) < 0.0001 ? estimateZ(velocity) : z);
        this.x = (Math.abs(x) < 0.0001 ? estimateX(velocity) : x);

        double error = -this.yaw + 2 * this.horizontalOffset;

        System.out.println(horizontalOffset + " " + this.horizontalOffset + " " + yaw + " " + this.yaw + " " + z + " "
                + this.z + " " + x + " " + this.x + " " + kP * error);

        return kP * error;
    }

    private double estimateYaw(double velocity) {
        double distanceSquared = getDistanceSquared();
        double previousX = x;
        double previousYaw = yaw;

        return previousYaw + dt * Math.toDegrees((velocity / distanceSquared) * 2 * previousX);
    }

    private double estimateZ(double velocity) {
        double previousZ = z;
        double previousYaw = yaw;

        return previousZ + dt * velocity * Math.sin(previousYaw);
    }

    private double estimateX(double velocity) {
        double previousX = x;
        double previousYaw = yaw;

        return previousX + dt * velocity * Math.sin(previousYaw);
    }

    private double getDistanceSquared() {
        return Math.pow(z, 2) + Math.pow(x, 2);
    }

    /**
     * Sets sources to rate mode and returns average of current positions
     * 
     * @return average of sensors' current rate
     */
    private double getSourcesAverageRate() {
        PIDSource leftSource = sources[0];
        PIDSource rightSource = sources[1];

        leftSource.setPIDSourceType(PIDSourceType.kRate);
        rightSource.setPIDSourceType(PIDSourceType.kRate);

        double average = (leftSource.pidGet() + rightSource.pidGet()) / 2;

        return average;
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
