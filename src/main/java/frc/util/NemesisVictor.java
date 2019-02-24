/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Custom VictorSPX class that implements PIDOutput
 * @author Harsh Padhye
 */
public class NemesisVictor extends VictorSPX implements PIDOutput{

    public NemesisVictor(int device) {
        super(device);
    }

    @Override
    public void pidWrite(double output) {
        set(ControlMode.PercentOutput, output);
    }

}
