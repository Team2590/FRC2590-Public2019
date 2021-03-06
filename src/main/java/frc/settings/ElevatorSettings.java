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
    public static final double ELEVATOR_KP = 0.6;
    public static final double ELEVATOR_KI = 0.12;
    public static final double ELEVATOR_KV = 1.0 / 57.0; // 45.5 in/s free speed
    public static final double ELEVATOR_KA = 0.003;

    public static final double ELEVATOR_MAX_VEL = 55.0; //40 in w/ new shaft //55.0
    public static final double ELEVATOR_MAX_ACC = 300.0; //175 in/s with new shaft //300
    public static final double ELEVATOR_SLOW_VEL = 55.0;
    public static final double ELEVATOR_SLOW_ACC = 300.0;
    public static final double ELEVATOR_TOLERANCE = 2.0;

    public static final double ELEVATOR_HOLD_CONSTANT = 0.25; //0.25
    public static final double ELEVATOR_CURRENT_CONTROL_CONSTANT = 0.3;

    public static final double GEAR_RATIO = 8.18;
    public static final double ENCODER_CONVERSION = 0.00216; // 0.001238 current

}
