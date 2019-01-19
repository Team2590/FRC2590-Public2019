/**
 * Code for Nemesis 2590's 2019 Robot
 * 
 * @author Harsh Padhye
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.subsystems.CargoIntake;
import frc.subsystems.Drivetrain;
import frc.subsystems.Elevator;
import frc.subsystems.HatchIntake;
import frc.util.NemesisJoystick;


public class Robot extends TimedRobot {

  private NemesisJoystick leftJoystick;
  private NemesisJoystick rightJoystick;

  // susbsystems
  private static Drivetrain drivetrain;
  private static HatchIntake hatchIntake;
  private static CargoIntake cargoIntake;
  private static Elevator elevator;

  /**
   * Initialization of robot, when robot is turned on
   */
  @Override
  public void robotInit() {
    leftJoystick = new NemesisJoystick(0);
    rightJoystick = new NemesisJoystick(1);

    drivetrain = Drivetrain.getDrivetrainInstance();
    hatchIntake = HatchIntake.getHatchIntakeInstance();
    cargoIntake = CargoIntake.getCargoIntakeInstance();
    elevator = Elevator.getElevatorInstance();
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
    
    // inverts Y axis; pushing the joystick forward drives forward
    drivetrain.teleopDrive(-leftJoystick.getY() * 0.5, rightJoystick.getX() * 0.5);

    // Hatch Intake controls
    if (rightJoystick.getRawButton(1)) {
      hatchIntake.runIntake();
    } else if (rightJoystick.getRawButton(2)) {
      hatchIntake.reverseIntake();
    } else {
      hatchIntake.stopIntake();
    }

    // updates each subsystem at the tail end of each loop
    hatchIntake.update();
    drivetrain.update();
  }

  /**
   * Test mode
   */
  @Override
  public void testPeriodic() {
  }

  // get methods for subsystems
  public static Drivetrain getDrivetrainInstance() {
    return drivetrain;
  }

  public static HatchIntake getHatchIntakeInstance() {
    return hatchIntake;
  }

  public static CargoIntake getCargoIntakeInstance() {
    return cargoIntake;
  }

  public static Elevator getElevatorInstance() {
    return elevator;
  }

}
