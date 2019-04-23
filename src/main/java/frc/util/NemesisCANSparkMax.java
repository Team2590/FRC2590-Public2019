/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import com.revrobotics.CANSparkMax;

/**
 * Custom CANSparkMax class
 */
public class NemesisCANSparkMax extends CANSparkMax{

    public NemesisCANSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
    }


}