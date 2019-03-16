/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class NemesisPotentiometer extends AnalogPotentiometer {

    private double slipLimit = 360.0;
    private double lastAngle, lastTime;

    public NemesisPotentiometer(int port) {
        super(port);
        lastAngle = get();
        lastTime = Timer.getFPGATimestamp();
    }

    public NemesisPotentiometer(int port, double scale) {
        super(port, scale);
        lastAngle = get();
    }

    public void setSlipLimit(double limit) {
        slipLimit = limit;
    }

    @Override
    public double get() {
        if (super.get() > slipLimit) {
            return super.get() - 360.0;
        }
        return super.get();
    }

    /**
     * Units is deg/sec
     * @return rotational rate
     */
    public double getRate(){
        double rate = (get() - lastAngle) / (Timer.getFPGATimestamp() - lastTime);
        lastAngle = get();
        lastTime = Timer.getFPGATimestamp();
        return rate;
    }

    @Override
    public double pidGet() {
        return this.get();
    }
}
