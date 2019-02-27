/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.settings;

/**
 * Organizes the gains & misc values for the elevator
 */
public interface ElevatorSettings {

    // Motion Profile Constants
    public static final double ELEVATOR_KP = 0.1;
    public static final double ELEVATOR_KI = 0.02;
    public static final double ELEVATOR_KV = 1.0 / (26.1 * 1.13); // 26.1 in/s free speed (1.13 is the scaling)
    public static final double ELEVATOR_KA = 0.0;

    public static final double ELEVATOR_MAX_VEL = 20.0;
    public static final double ELEVATOR_MAX_ACC = 40.0;
    public static final double ELEVATOR_TOLERANCE = 2.0;

    public static final double ELEVATOR_HOLD_CONSTANT = 0.25;

    public static final double GEAR_RATIO = 8.18;

}
