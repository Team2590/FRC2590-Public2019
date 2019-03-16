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

    // Operator Joystick
    
    public static final int ELEVATOR_CARGO_SHIP = 1;
    public static final int FORCE_TELEOP = 2;
    public static final int CARRIAGE_FRONT = 4;
    public static final int CARRIAGE_MIDDLE = 5;
    public static final int CARRIAGE_BACK = 6;

}
