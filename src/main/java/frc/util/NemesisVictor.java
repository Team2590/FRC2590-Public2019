/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Add your docs here.
 */
public class NemesisVictor extends VictorSPX implements PIDOutput{

    public NemesisVictor(int device) {
        super(device);
    }

    @Override
    public void pidWrite(double output) {

    }
}
