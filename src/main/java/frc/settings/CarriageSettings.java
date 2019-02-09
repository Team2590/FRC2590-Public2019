/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.settings;

/**
 * Organizes the gains & misc values for the carriage
 */
public interface CarriageSettings {

    //Motion Profile Constants
    public static final double CARRIAGE_KP = 0.0;
    public static final double CARRIAGE_KI = 0.0;
    public static final double CARRIAGE_KV = 0.0;
    public static final double CARRIAGE_KA = 0.0;

    public static final double CARRIAGE_MAX_VEL = 0.0;
    public static final double CARRIAGE_MAX_ACC = 0.0;
    public static final double CARRIAGE_TOLERANCE = 0.0;
}