/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

/**
 * Add your docs here.
 */
public class SoftStop {

    private double maxCurrent, supplyVoltage, resistance, kP;
    // 40 amps, 12 V,

    public SoftStop(double maxCurrent, double supplyVoltage, double resistance, double kP) {
        this.maxCurrent = maxCurrent;
        this.supplyVoltage = supplyVoltage;
        this.resistance = resistance;
        this.kP = kP;
    }

    public double calculateOffset(double motorCurrent) {
        if (motorCurrent > maxCurrent) {
            double deltaCurrent = motorCurrent - maxCurrent;
            return (resistance / supplyVoltage) * deltaCurrent * kP;
        } else {
            return 0.0;
        }
    }
}
