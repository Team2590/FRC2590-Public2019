
package frc.robot;

/**
 * Interface to organize the various ports, gains, PWM Channels, CAN IDs
 */
public interface RobotMap {

    // Drivetrain
    public static final int LEFT_DRIVE_MASTER = 1;
    public static final int LEFT_DRIVE_SLAVE = 2;
    public static final int RIGHT_DRIVE_MASTER = 3;
    public static final int RIGHT_DRIVE_SLAVE = 4;

    public static final int LEFT_DRIVE_ENCODER_A = 2;
    public static final int LEFT_DRIVE_ENCODER_B = 3;
    public static final int RIGHT_DRIVE_ENCODER_A = 4;
    public static final int RIGHT_DRIVE_ENCODER_B = 5;

    public static final int GEAR_SHIFT_SOLENOID = 7;

    public static final int DRIVETRAIN_GYRO = 0;

    // Cargo intake
    public static final int CARGO_ARTICULATOR = 6;
    public static final int CARGO_INTAKE = 7;

    // Hatch Intake
    public static final int HATCH_INTAKE = 8;
    public static final int INTAKE_SOLENOID = 6;

    public static final int CARGO_POTENTIOMETER = 2;

    // Elevator
    public static final int ELEVATOR_MOTOR = 5;

    public static final int ELEVATOR_ENCODER_A = 0;
    public static final int ELEVATOR_ENCODER_B = 1;

    // Carriage
    public static final int ARM_SOLENOID = 0;
    public static final int BCV_EXTENDER_SOLENOID = 2;
    public static final int BCV_FINGER_SOLENOID = 1;

    public static final int LEFT_CARRIAGE_MOTOR = 9;
    public static final int RIGHT_CARRIAGE_MOTOR = 10;
    public static final int SWING_CARRIAGE_MOTOR = 11;

    public static final int CARRIAGE_POTENTIOMETER = 1;
}
