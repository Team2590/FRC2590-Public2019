
package frc.settings;

/**
 * Organizes the gains & misc values for the drivetrain
 */
public interface DrivetrainSettings {

    public static final int MAX_LOW_GEAR_VELOCITY = 7;

    // drivetrain dimensions (inches)
    public static final double WHEEL_DIAMETER = 6.0;
    public static final double WHEEL_BASE_WIDTH = 27.25;

    // gains for motion profiling
    public static final double DRIVETRAIN_KP = 0.0;
    public static final double DRIVETRAIN_KI = 0.0;
    public static final double DRIVETRAIN_KV = 1/233.4; // 233.4 in/s free speed
    public static final double DRIVETRAIN_KA = 0.0;

    // gains for turning in place
    public static final double TURN_KP = 0.0;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;
}
