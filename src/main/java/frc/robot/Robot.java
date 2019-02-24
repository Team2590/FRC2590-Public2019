/**
 * Code for Nemesis 2590's 2019 Robot
 * 
 * @author Harsh Padhye
 */
package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.looper.Looper;
import frc.settings.FieldSettings;
import frc.subsystems.CargoIntake;
import frc.subsystems.Carriage;
import frc.subsystems.Drivetrain;
import frc.subsystems.Elevator;
import frc.subsystems.HatchIntake;
import frc.util.Limelight;
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

  //PDP for current draw purposes
  private PowerDistributionPanel pdp;

  // Limelight camera for vision targeting
  private static Limelight limelight;

  // Compressor
  private Compressor compressor;

  // Looper holster for multithreaded control of subsystems
  private Looper enabledLooper;

  // controls if the elevator setpoints correlate to hatch heights or ball heights
  // true if hatch heights, false if cargo heights
  private boolean hatchButtonMode;

  /**
   * Initialization of robot, when robot is turned on
   */
  @Override
  public void robotInit() {
    // joysticks, 10% deadbands on X and Y for each
    leftJoystick = new NemesisJoystick(0, 0.1, 0.1);
    rightJoystick = new NemesisJoystick(1, 0.1, 0.1);
    operatorJoystick = new NemesisJoystick(2, 0.1, 0.1);

    // instantiate subsystems
    drivetrain = Drivetrain.getDrivetrainInstance();
    hatchIntake = HatchIntake.getHatchIntakeInstance();
    cargoIntake = CargoIntake.getCargoIntakeInstance();
    carriage = Carriage.getCarriageInstance();
    elevator = Elevator.getElevatorInstance();

    pdp = new PowerDistributionPanel();

    // limelight camera
    limelight = Limelight.getLimelightInstance();

    enabledLooper = new Looper(REFRESH_RATE);

    // add subsystems to the Looper holster
    enabledLooper.register(drivetrain::update);
    enabledLooper.register(hatchIntake::update);
    // enabledLooper.register(cargoIntake::update);
    enabledLooper.register(carriage::update);
    enabledLooper.register(elevator::update);

    // instantiates compressor
    compressor = new Compressor();
    compressor.clearAllPCMStickyFaults();

    // default mode is hatch mode
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
    enabledLooper.startLoops();
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
    enabledLooper.startLoops();
    drivetrain.resetAllSensors();
    cargoIntake.holdPosition();
    carriage.holdPosition();
  }

  /**
   * Teleoperated Mode, Driver controls robot
   */
  @Override 
  public void teleopPeriodic() {

    // System.out.println("PDP :: " + pdp.getCurrent(4));
    // System.out.println("Carriage :: " + carriage.getAngle());
    // System.out.println("Cargo Intake :: " + cargoIntake.getAngle());

    // This logic checks if the robot is still turning, to avoid switching states to
    // teleop in the middle of the control loop
    // Also prevents driver from moving the robot while it auto aligns
    // if (drivetrain.isTurnDone()) {
    drivetrain.teleopDrive(-leftJoystick.getYBanded(), rightJoystick.getXBanded());
    // }

    // Auto Align
    // input the limelight reading once to avoid latency issues

    /*
     * if (leftJoystick.getRisingEdge(1)) { limelight.update(); double
     * visionSetpoint = limelight.horizontalAngleToTarget();
     * drivetrain.turn(-visionSetpoint); }
     */

    // Runs the cargo intake
    if (leftJoystick.getRawButton(1)) {
      cargoIntake.runIntake();
    } else {
      cargoIntake.stopIntake();
    }

    // moves cargo intake to designated setpoints
    if (rightJoystick.getRisingEdge(1)) {
      cargoIntake.bottomPosition();
    } else if (rightJoystick.getRisingEdge(2)) {
      cargoIntake.topPosition();
    }

    // Runs the carriage intake
    if (leftJoystick.getRawButton(2)) {
      carriage.spinArmWheels(1.0);
    } else if (leftJoystick.getRawButton(3)) {
      carriage.spinArmWheels(-1.0);
    } else {
      carriage.spinArmWheels(0.0);
    }

    // swings the carriage
    if (operatorJoystick.getRisingEdge(4)) {
      //hatchIntake.drop();
      carriage.frontPosition();
    } else if (operatorJoystick.getRisingEdge(5)) {
      carriage.uprightPosition();
    } else if(operatorJoystick.getRisingEdge(6)) {
      carriage.backPosition();
    }

    // moves the cargo intake manually
    if (operatorJoystick.getPOV() == 0) {
      // cargoIntake.moveManually(0.5);
      carriage.manualSwing(-0.45);
    } else if (operatorJoystick.getPOV() == 180) {
      // cargoIntake.moveManually(-0.5);
      carriage.manualSwing(0.45);
    }

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

    // moves the elevator manually using the hat on the joystick
    if (rightJoystick.getPOV() == 0) {
      elevator.moveManually(0.5);
    } else if (rightJoystick.getPOV() == 180) {
      elevator.moveManually(-0.3);
    }

    // switches between hatch and ball elevator setpoints
    if (leftJoystick.getRisingEdge(4)) {
      hatchButtonMode = true; // hatch panel
    } else if (leftJoystick.getRisingEdge(5)) {
      hatchButtonMode = false; // cargo ball
    }

    // PNEUMATIC TEST
    // lifts and drops hatch dustpan
    if (operatorJoystick.getRisingEdge(1)) {
      hatchIntake.toggleDustpan();
    }

    /*
     * extends the BCV slide piston if (operatorJoystick.getRawButton(3)) {
     * carriage.extendBCV(); } else { carriage.retractBCV(); }
     */

    // opens BCV fingers only when arms are open
    if (operatorJoystick.getRawButton(2)) {
      carriage.openArms();
      if (operatorJoystick.getRawButton(3)) {
        carriage.openBCV();
      } else {
        carriage.closeBCV();
      }
    } else {
      carriage.closeArms();
    }

    // Hatch Intake controls for Intaking, Spitting, and Stopping
    if (operatorJoystick.getRawButton(7)) {
      hatchIntake.runIntake();
    } else {
      hatchIntake.stopIntake();
    }

  }

  @Override
  public void disabledPeriodic() {
    enabledLooper.endLoops();
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

  public static Limelight getLimelightInstance() {
    return limelight;
  }

}
