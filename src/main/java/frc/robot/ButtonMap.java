
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Easy access interface for the joystick button mappings
 */
public interface ButtonMap {

    // Left Joystick
    public static final int AUTO_ALIGN = 1;
    public static final int DOWNSHIFT = 2;
    public static final int UPSHIFT = 3;
    public static final int HATCH_MODE = 4;
    public static final int BALL_MODE = 5;

    // Right Joystick
    public static final int BALL_INTAKE = 1;
    public static final int BALL_OUTTAKE = 2;

    public static final int CLOSE_BCV = 1;

    public static final int ELEVATOR_LOW = 5;
    public static final int ELEVATOR_MID = 6;
    public static final int ELEVATOR_HIGH = 4;
    public static final int ELEVATOR_GROUND = 3;

    // Operator Joystick (10 is dead)
    public static final int LEVEL_3_CLIMBER_AND_ELEVATOR = 1;
    public static final int CLIMBER_INTAKE = 2;
    public static final int ELEVATOR_CARGO_SHIP = 3;
    public static final int CARRIAGE_FRONT = 4;
    public static final int CARRIAGE_MIDDLE = 5;
    public static final int CARRIAGE_BACK = 6;
    public static final int CARRIAGE_LATCH = 7;
    public static final int ELEVATOR_HARDSTOP_DISABLE = 8;
    public static final int FORCE_TELEOP = 9;
    public static final int FORCE_TELEOP_FAILSAFE = 10;
    public static final int CARRIAGE_FRONT_HATCH = 11;
    public static final int CARRIAGE_PLACE_HATCH = 12;

}
