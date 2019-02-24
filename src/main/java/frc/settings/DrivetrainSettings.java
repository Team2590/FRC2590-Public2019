
package frc.settings;

/**
 * Organizes the gains & misc values for the drivetrain
 */
public interface DrivetrainSettings {

    // max speeds of the DT in ft/s, converted to in/s
    public static final double MAX_LOW_GEAR_VELOCITY = 5.53 * 12;
    public static final double MAX_HIGH_GEAR_VELOCITY = 15.75 * 12;

    public static final double HIGH_GEAR_RATIO = 7.64;
    public static final double LOW_GEAR_RATIO = 15.71;

    // drivetrain dimensions (inches)
    public static final double WHEEL_DIAMETER = 6.0;
    public static final double WHEEL_BASE_WIDTH = 27.25;

    // gains for drive motor linearizartion
    public static final double LEFT_LINEAR_KP = 0.0;
    public static final double LEFT_LINEAR_KI = 0.0;
    public static final double RIGHT_LINEAR_KP = 0.0;
    public static final double RIGHT_LINEAR_KI = 0.0;

    // gains for motion profiling
    public static final double DRIVETRAIN_KP = 0.0;
    public static final double DRIVETRAIN_KI = 0.0;
    public static final double DRIVETRAIN_KV = 1.0 / 233.4; // 233.4 in/s free speed
    public static final double DRIVETRAIN_KA = 0.0;

    // gains for turning in place
    public static final double TURN_KP = 0.0;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;
}
