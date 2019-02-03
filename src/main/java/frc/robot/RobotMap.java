
package frc.robot;

/**
 * Interface to organize the various ports, gains, PWM Channels, CAN IDs
 */
public interface RobotMap {
    
    // Drivetrain
    public static final int LEFT_DRIVE_MASTER = 1;
    public static final int LEFT_DRIVE_SLAVE = 2;
    public static final int RIGHT_DIRVE_MASTER = 3;
    public static final int RIGHT_DRIVE_SLAVE = 4;

    public static final int GEAR_SHIFT_SOLENOID = 1;


    // Cargo intake
    public static final int CARGO_INTAKE = 6;


    // Elevator
    public static final int ELEVATOR_MOTOR = 7;


    // Hatch Intake and TCV
    public static final int HATCH_INTAKE = 5;
    public static final int INTAKE_SOLENOID = 4;
    public static final int BCV_SOLENOID = 3;
}
