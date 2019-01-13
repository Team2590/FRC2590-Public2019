
package frc.robot;

/**
 * Interface to organize the various ports, gains, PWM Channels, CAN IDs
 */
public interface RobotMap {
    //CAN Device IDs for the drive motor
    public static final int left_drive_master  = 4;
    public static final int left_drive_slave = 1;
    public static final int right_drive_master = 2;
    public static final int right_drive_slave = 3;
}
