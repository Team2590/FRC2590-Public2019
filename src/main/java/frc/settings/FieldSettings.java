/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.settings;

/**
 * Holster for the field geometries, FMS variables, etc.
 */
public interface FieldSettings {

    // setpoints for the hatch heights on the Rocket Ship
    public static final double ROCKET_LOW_HATCH = 9.5;
    public static final double ROCKET_MID_HATCH = 37.5;
    public static final double ROCKET_HIGH_HATCH = 60.5; // should be 75, lower for testing purposes

    // setpoints for the cargo heights on the Rocket Ship
    public static final double ROCKET_LOW_CARGO = 15.5;
    public static final double ROCKET_MID_CARGO = 43.5;
    public static final double ROCKET_HIGH_CARGO = 59.5; // should be 75, lower for testing purposes

    // setpoints for the hatch and cargo heights on the Cargo Ship
    public static final double CARGO_SHIP_HATCH = 0.0;
    public static final double CARGO_SHIP_CARGO = 0.0;

    // refresh rate for the Looper class
    public static final double REFRESH_RATE = 0.02; // 10ms
}
