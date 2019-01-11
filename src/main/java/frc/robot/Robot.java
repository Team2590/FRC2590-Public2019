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

  Joystick leftJoystick;
  Joystick rightJoystick;

  //susbsystems
  Drivetrain drivetrain;
  
  /**
   * Initialization of robot, when robot is turned on
   */
  @Override
  public void robotInit() {
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
    drivetrain.setSpeeds(leftJoystick.getY() * .33, rightJoystick.getX() * .33);
  }

  /**
   * Test mode
   */
  @Override
  public void testPeriodic() {
  }

  public Drivetrain getDrivetrain() {
    return drivetrain;
  }
}
