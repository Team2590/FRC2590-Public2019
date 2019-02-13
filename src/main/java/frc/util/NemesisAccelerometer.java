/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.wpilibj.AnalogAccelerometer;

/**
 * Custom accelerometer class to account for linear acceleration
 * @author Harsh Padhye
 */
public class NemesisAccelerometer extends AnalogAccelerometer {

    private AnalogAccelerometer offset;

    public NemesisAccelerometer(int port) {
        super(port);
    }

    public NemesisAccelerometer(int port, AnalogAccelerometer offset) {
        super(port);
        this.offset = offset;
    }

    @Override
    public double pidGet() {
        if (offset == null) {
            return this.getAcceleration();
        } else {
            return this.getAcceleration() - offset.getAcceleration();
        }
    }

}
