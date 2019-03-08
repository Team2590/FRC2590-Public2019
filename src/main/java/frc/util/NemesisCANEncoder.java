/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Same as CANEncoder, but implements PIDSource for use in motion controllers
 * 
 * @author Harsh Padhye
 */
public class NemesisCANEncoder extends CANEncoder implements PIDSource {

    private boolean reverseDirection = false;
    private double scaling = 1.0;

    public NemesisCANEncoder(com.revrobotics.CANSparkMax device) {
        super(device);
    }

    public void setScaling(double scaling) {
        this.scaling = scaling;
    }

    @Override
    public double getPosition() {
        return super.getPosition() * (reverseDirection ? -1 : 1) * scaling;
    }

    @Override
    public double getVelocity() {
        return super.getVelocity() * (reverseDirection ? -1 : 1) * scaling;
    }

    @Override
    public double pidGet() {
        return this.getPosition();
    }

    public void setReverseDirection(boolean reverse) {
        reverseDirection = reverse;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return null;
    }
}
