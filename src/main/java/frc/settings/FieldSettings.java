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
    public static final double ROCKET_LOW_HATCH = 4.5;
    public static final double ROCKET_MID_HATCH = 32.5;
    public static final double ROCKET_HIGH_HATCH = 62.0; // should be 75, lower for testing purposes

    // setpoints for the cargo heights on the Rocket Ship
    public static final double ROCKET_LOW_CARGO = 9.5;
    public static final double ROCKET_MID_CARGO = 38.0;
    public static final double ROCKET_HIGH_CARGO = 64.0; // should be 75, lower for testing purposes

    // setpoint for feeder station
    public static final double FEEDER_STATION = 35.5;

    // setpoint for latching the carriage
    public static final double LATCH_HEIGHT = 20.75;  //previously 20, halved for tuning purposes

    // refresh rate for the Looper class
    public static final double REFRESH_RATE = 0.02; // 10ms
}
