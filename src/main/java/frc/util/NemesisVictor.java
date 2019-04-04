/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import java.util.LinkedList;
import java.util.Queue;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;

/**
 * Custom VictorSPX class that implements PIDOutput
 * 
 * @author Harsh Padhye
 */
public class NemesisVictor extends VictorSPX implements PIDOutput {

    private Queue<Double> heatSum;
    private Queue<Double> timeSum;
    private PowerDistributionPanel pdp;
    private int pdpChannel;
    private double lastTime, totalHeat, lastCurrent, totalTime, rollingAvgTime, heatThreshold;
    private boolean burnoutProtection, bpEnabled;

    public NemesisVictor(int device) {
        super(device);
        heatSum = new LinkedList<>();
        timeSum = new LinkedList<>();
        pdp = new PowerDistributionPanel();
        lastTime = Timer.getFPGATimestamp();
        lastCurrent = 0.0;
        totalTime = 0.0;
        totalHeat = 0.0;
        burnoutProtection = false;
        bpEnabled = false;
    }

    public NemesisVictor(int device, int pdpChannel, boolean enableBurnoutProtection, double rollingAvgTime,
            double heatThreshold) {
        this(device);
        burnoutProtection = enableBurnoutProtection;
        this.pdpChannel = pdpChannel;
        this.rollingAvgTime = rollingAvgTime;
        this.heatThreshold = Math.pow(heatThreshold, 2);
    }

    @Override
    public void pidWrite(double output) {
        if (!burnoutProtection || !checkCurrent()) {
            if (bpEnabled) {
                bpEnabled = false;
                System.out.printf("Burnout Protection Disabled for PDP Channel %d \n", pdpChannel);
            }
            set(ControlMode.PercentOutput, output);
        } else {
            if (!bpEnabled) {
                bpEnabled = true;
                System.out.printf("Burnout Protection Enabled for PDP Channel %d \n", pdpChannel);
            }
            System.out.printf("im in burnout mode \n");
            set(ControlMode.PercentOutput, output * 0.2);
        }
    }

    public boolean checkCurrent() {
        double currentTime = Timer.getFPGATimestamp();
        double currentCurrent = pdp.getCurrent(pdpChannel);
        double deltaT = currentTime - lastTime;
        timeSum.add(deltaT);
        totalTime += deltaT;

        double heat = Math.abs(Math.pow(((currentCurrent + lastCurrent) / 2), 2) * deltaT);
        heatSum.add(heat);
        totalHeat += heat;

        while (totalTime > rollingAvgTime) {
            totalTime -= timeSum.remove();
            totalHeat -= heatSum.remove();
        }

        lastTime = currentTime;
        lastCurrent = currentCurrent;

        double motorHeat = totalHeat / totalTime;
        if (totalTime == 0) {
            motorHeat = 0.0;
        }
        // System.out.printf("heat total %.02f heat threshold %.02f pdp current
        // %.02f\n", Math.pow(motorHeat, 0.5),
        // Math.pow(heatThreshold, 0.5), currentCurrent);
        return (motorHeat > heatThreshold);
    }

}
