
package frc.robot;

/**
 * Interface to organize the various ports, gains, PWM Channels, CAN IDs
 */
public interface RobotMap {
    // CAN Device IDs for the drive motor
    public static final int left_drive_master = 4;
    public static final int left_drive_slave = 1;
    public static final int right_drive_master = 2;
    public static final int right_drive_slave = 3;

    // CAN Device IDs for the intake motors
    public static final int hatch_intake = 5;
    public static final int cargo_intake = 6;

    // CAN Device IDs for the elevator motors
    public static final int elevator_motor = 7;

    //Solenoid channel for Hatch Intake
    public static final int intake_solenoid = 4;
}
