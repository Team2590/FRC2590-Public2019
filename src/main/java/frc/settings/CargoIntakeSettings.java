/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.settings;

/**
 * Organizes the gains & misc values for the cargo intake
 */
public interface CargoIntakeSettings {

    // Motion Profile / PID Constants
    public static final double CARGO_INTAKE_KP = 0.075;
    public static final double CARGO_INTAKE_KI = 0.0;
    public static final double CARGO_INTAKE_KD = 0.002;

    public static final double CARGO_INTAKE_KV = 0.01; // 766.1 deg/s free speed
    public static final double CARGO_INTAKE_KA = 0.001;

    public static final double CARGO_INTAKE_MAX_VEL = 120.0;
    public static final double CARGO_INTAKE_MAX_ACC = 480.0;
    public static final double CARGO_INTAKE_TOLERANCE = 0.0;

    public static final double kP_HOLD_CONSTANT = 0.025;
    public static final double kI_HOLD_CONSTANT = 0.005;
    public static final double kD_HOLD_CONSTANT = 0.0;

    public static final double TOP_POSITION = 5.0;
    public static final double BOTTOM_POSITION = 60.0;
}
