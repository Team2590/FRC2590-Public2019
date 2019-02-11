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

    // Motion Profile Constants
    public static final double CARGO_INTAKE_KP = 0.0;
    public static final double CARGO_INTAKE_KI = 0.0;
    public static final double CARGO_INTAKE_KV = 1/2675; // 2675 deg/s free speed
    public static final double CARGO_INTAKE_KA = 0.0;

    public static final double CARGO_INTAKE_MAX_VEL = 0.0;
    public static final double CARGO_INTAKE_MAX_ACC = 0.0;
    public static final double CARGO_INTAKE_TOLERANCE = 0.0;
}
