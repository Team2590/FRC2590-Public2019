/**
 * Code for Nemesis 2590's 2019 Robot
 * 
 * @author Harsh Padhye
 */
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.subsystems.Drivetrain;


public class Robot extends TimedRobot {

  private Joystick leftJoystick;
  private Joystick rightJoystick;

  //susbsystems
  private static Drivetrain drivetrain;
  
  /**
   * Initialization of robot, when robot is turned on
   */
  @Override
  public void robotInit() {
    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);

    drivetrain = Drivetrain.getDrivetrainInstance();
  }

  @Override
  public void robotPeriodic() {
  }

  /**
   * Initialization of autonomous mode, Right before match begins
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * Autonomous Mode, Robot moves on its own
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * Initialization of teleoperated mode, Switches from auto to teleop
   */
  @Override
  public void teleopInit() {
  }
  /**
   * Teleoperated Mode, Driver controls robot
   */
  @Override
  public void teleopPeriodic() {
    if(leftJoystick == null) {
      System.out.println("left joy is null");
    }
    if(rightJoystick == null) {
      System.out.println("right joy is null");
    }
    if(drivetrain == null) {
      System.out.println("drivetrain is null");
    }
    if(drivetrain.getRobotDrive() == null){
      System.out.println("diffdrive is null");
    }

    drivetrain.getRobotDrive().arcadeDrive(leftJoystick.getY() * .33, rightJoystick.getX() * .33);
  }

  /**
   * Test mode
   */
  @Override
  public void testPeriodic() {
  }

  public static Drivetrain getDrivetrainInstance() {
    return drivetrain;
  }
}
