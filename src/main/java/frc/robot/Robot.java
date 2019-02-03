/**
 * Code for Nemesis 2590's 2019 Robot
 * 
 * @author Harsh Padhye
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.settings.FieldSettings;
import frc.subsystems.CargoIntake;
import frc.subsystems.Carriage;
import frc.subsystems.Drivetrain;
import frc.subsystems.Elevator;
import frc.subsystems.HatchIntake;
import frc.util.NemesisJoystick;

public class Robot extends TimedRobot implements FieldSettings {

  // joysticks
  private NemesisJoystick leftJoystick;
  private NemesisJoystick rightJoystick;
  private NemesisJoystick operatorJoystick;

  // susbsystems
  private static Drivetrain drivetrain;
  private static HatchIntake hatchIntake;
  private static CargoIntake cargoIntake;
  private static Carriage carriage;
  private static Elevator elevator;

  // controls if the elevator setpoints correlate to hatch heights or ball heights
  // true if hatch heights, false if cargo heights
  private boolean hatchButtonMode;

  /**
   * Initialization of robot, when robot is turned on
   */
  @Override
  public void robotInit() {
    leftJoystick = new NemesisJoystick(0);
    rightJoystick = new NemesisJoystick(1);
    operatorJoystick = new NemesisJoystick(2);

    drivetrain = Drivetrain.getDrivetrainInstance();
    hatchIntake = HatchIntake.getHatchIntakeInstance();
    cargoIntake = CargoIntake.getCargoIntakeInstance();
    carriage = Carriage.getCarriageInstance();
    elevator = Elevator.getElevatorInstance();

    hatchButtonMode = true;
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

    // Hatch Intake controls for Intaking, Spitting, and Stopping
    if (rightJoystick.getRawButton(1)) {
      hatchIntake.runIntake();
    } else if (rightJoystick.getRawButton(2)) {
      hatchIntake.reverseIntake();
    } else {
      hatchIntake.stopIntake();
    }

    // lifts and drops hatch dustpan
    if (rightJoystick.getRawButton(3)) {
      hatchIntake.drop();
    } else {
      hatchIntake.stow();
    }

    // opens and closes Bicuspid valve
    // if (rightJoystick.getRawButton(4)) {
    // carriage.openBCV();
    // } else {
    // carriage.closeBCV();
    // }

    // Elevator stepoint controls
    if (hatchButtonMode) { // hatch heights
      if (rightJoystick.getRisingEdge(5)) {
        elevator.moveSmooth(ROCKET_LOW_HATCH);

      } else if (rightJoystick.getRisingEdge(6)) {
        elevator.moveSmooth(ROCKET_MID_HATCH);

      } else if (rightJoystick.getRisingEdge(4)) {
        elevator.moveSmooth(ROCKET_HIGH_HATCH);
      }

    } else { // cargo ball heights
      if (rightJoystick.getRisingEdge(5)) {
        elevator.moveSmooth(ROCKET_LOW_CARGO);

      } else if (rightJoystick.getRisingEdge(6)) {
        elevator.moveSmooth(ROCKET_MID_CARGO);

      } else if (rightJoystick.getRisingEdge(4)) {
        elevator.moveSmooth(ROCKET_HIGH_CARGO);
      }

    }
    // drops the elevator to ground level, regardless of setpoint mode
    if (rightJoystick.getRisingEdge(3)) {
      elevator.moveSmooth(0.0);
    }

    // switches between hatch and ball elevator setpoints
    if (leftJoystick.getRisingEdge(4)) {
      hatchButtonMode = true; // hatch panel
    } else if (leftJoystick.getRisingEdge(5)) {
      hatchButtonMode = false; // cargo ball
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

  public static Carriage getCarriageInstance() {
    return carriage;
  }

  public static Elevator getElevatorInstance() {
    return elevator;
  }

}
