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

    // Motion Profile Constants
    public static final double CARRIAGE_KP = 0.05;
    public static final double CARRIAGE_KI = 0.01;
    public static final double CARRIAGE_KV = 1.0 / 290.5; // 290.5 deg/s free speed under load
    public static final double CARRIAGE_KA = 0.0015;

    public static final double CARRIAGE_MAX_VEL = 120.0;
    public static final double CARRIAGE_MAX_ACC = 240.0;
    public static final double CARRIAGE_TOLERANCE = 2.0;

    public static final double kP_HOLD_CONSTANT = 0.0275; //0.0275
    public static final double kI_HOLD_CONSTANT = 0.005; //0.005
    public static final double kD_HOLD_CONSTANT = 0.0001;

    public static final double FRONT_POSITION = 180.0;
    public static final double BACK_POSITION = 7.0; //not set to zero to that it doest overflow to 360
    public static final double HATCH_HANDOFF_POSITION = 0.0;
    public static final double UPRIGHT_POSITION = 90.0;
}
