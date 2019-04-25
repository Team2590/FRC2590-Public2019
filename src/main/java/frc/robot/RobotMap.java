
package frc.robot;

/**
 * Interface to organize the various ports, gains, PWM Channels, CAN IDs
 */
public interface RobotMap {

    /**
     * Drivetrain
     */
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

    //Drivetrain pdp channels
    public static final int LM_PWM = 1;
    public static final int LS_PWM = 2;
    public static final int RM_PWM = 12;
    public static final int RS_PWM = 15;

    /**
     * Climber
     */
    public static final int CLIMBER_ARTICULATE_MASTER = 6;
    public static final int CLIMBER_ARTICULATE_SLAVE = 7; //7 on real bot, 9 on pbot
    public static final int CLIMBER_INTAKE = 8;
    public static final int CLIMBER_POTENTIOMETER = 2;


    /**
     * Elevator
     */
    public static final int ELEVATOR_MOTOR = 5;

    public static final int ELEVATOR_ENCODER_A = 0;
    public static final int ELEVATOR_ENCODER_B = 1;

    public static final int ELEVATOR_HARDSTOP_SOLENOID = 6;

    /**
     * 
     * Carriage
     */
    public static final int BCV_FINGER_SOLENOID = 1; //1 for real bot, 2 for pbot

    public static final int INTAKE_CARRIAGE_MOTOR = 10;
    public static final int SWING_CARRIAGE_MOTOR = 11;

    public static final int CARRIAGE_POTENTIOMETER = 1;
}
