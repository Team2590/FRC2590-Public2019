/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.settings;

/**
 * Add your docs here.
 */
public interface ClimberSettings {

    // Motion Profile / PID Constants
    public static final double CLIMBER_KP = 0.0;
    public static final double CLIMBER_KI = 0.0;

    public static final double CLIMBER_KV = 0.0;
    public static final double CLIMBER_KA = 0.0;

    public static final double CLIMBER_MAX_VEL = 0.0;
    public static final double CLIMBER_MAX_ACC = 0.0;
    public static final double CLIMBER_TOLERANCE = 5.0;

    public static final double kP_HOLD_CONSTANT = 0.0;
    public static final double kI_HOLD_CONSTANT = 0.0;
    public static final double kD_HOLD_CONSTANT = 0.0;

    public static final double TOP_POSITION = 7.0; // change back to 5.0 deg
    public static final double BOTTOM_POSITION = 68.0; // 65.0
}
