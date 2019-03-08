/**
 * Code for Nemesis 2590's 2019 Robot
 * 
 * @author Harsh Padhye
 */
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
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

  // PDP for current draw purposes
  private PowerDistributionPanel pdp;

  // Limelight camera for vision targeting
  private static Limelight limelight;

  private CameraServer camServer;

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

    /*
     * camServer.addAxisCamera("10.25.90.12"); camServer.startAutomaticCapture();
     */

    enabledLooper = new Looper(REFRESH_RATE);

    // add subsystems to the Looper holster
    enabledLooper.register(drivetrain::update);
    enabledLooper.register(hatchIntake::update);
    enabledLooper.register(cargoIntake::update);
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
    drivetrain.resetAllSensors();
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

    /**
     * Useful print statements for measurement/calibration
     */
    // System.out.println("PDP :: " + pdp.getCurrent(4));
    // System.out.println("Carriage :: " + carriage.getAngle());
    // System.out.println("Cargo Intake :: " + cargoIntake.getAngle());

    // System.out.println("Elevator Encoder :: " + elevator.getHeight());
    // System.out.println(drivetrain.getDistanceNEO(true) + " " +
    // drivetrain.getDistanceQuadEnc(true) + " "
    // + drivetrain.getDistanceNEO(false) + " " +
    // drivetrain.getDistanceQuadEnc(false));

    // This logic checks if the robot is still turning, to avoid switching states to
    // teleop in the middle of the control loop
    // Also prevents driver from moving the robot while it auto aligns
    // if (drivetrain.isTurnDone()) {
    drivetrain.teleopDrive(-leftJoystick.getYBanded(), rightJoystick.getXBanded());
    // }

    // Auto Align
    // input the limelight reading once to avoid latency issues

    if (leftJoystick.getRisingEdge(1)) {
      limelight.update();
      drivetrain.turn(drivetrain.getHeading() + limelight.horizontalAngleToTarget());
    }

    // drops dustpan and outtakes slowly
    /*
     * if (leftJoystick.getRawButton(3)) { hatchIntake.dropDustpan();
     * hatchIntake.reverseIntake(); }
     */

    if (rightJoystick.getPOV() == 0) {
      elevator.moveManually(.25);
    } else if (rightJoystick.getPOV() == 180) {
      elevator.moveManually(-.25);
    }

    if (leftJoystick.getRisingEdge(3)) {
      hatchButtonMode = false;
      carriage.frontPosition();
    }

    // move cargo intake manually
    if (operatorJoystick.getPOV() == 0) {
      cargoIntake.moveManually(0.75);
    } else if (operatorJoystick.getPOV() == 180) {
      cargoIntake.moveManually(-0.75);
    }

    // Operator has the ability to move the carriage
    if (operatorJoystick.getRisingEdge(4)) {
      hatchButtonMode = false;
      carriage.frontPosition();
    } else if (operatorJoystick.getRisingEdge(5)) {
      hatchButtonMode = false;
      carriage.uprightPosition();
    } else if (operatorJoystick.getRisingEdge(6)) {
      hatchButtonMode = false;
      carriage.backPosition();
    }

    // moves cargo intake within frame perimeter
    if (operatorJoystick.getRisingEdge(3)) {
      cargoIntake.topPosition();
    }

    // Switches between Hatch Mode and Ball Mode
    if (leftJoystick.getRisingEdge(4)) {
      hatchButtonMode = true;
      elevator.moveSmooth(2.0);
      carriage.frontPosition();
    } else if (leftJoystick.getRisingEdge(5)) {
      hatchButtonMode = false;
      elevator.moveSmooth(2.0);
      carriage.backPosition();
    }

    /**
     * Here lies the logic for button mapping between hatch and cargo mode Some
     * buttons serve homologous functions between the two modes (eg: elevator
     * setpoint buttons)
     */

    // hatch mode
    if (hatchButtonMode) {
      // only opens arms if carriage is on front

      // switching to hatch mdoe should automatically move carriage
      // this acts as a safety precaution
      if (carriage.getCurrentOrientation()) {
        carriage.openArms();
      }

      // elevator setpoints
      // only allows driver to raise elevator while it is in the front position
      if (carriage.getCurrentOrientation()) {
        if (rightJoystick.getRisingEdge(3)) {
          // carriage.frontPosition();
          elevator.moveSmooth(2.0);
        } else if (rightJoystick.getRisingEdge(4)) {
          // carriage.frontPosition();
          elevator.moveSmooth(ROCKET_HIGH_HATCH);
        } else if (rightJoystick.getRisingEdge(5)) {
          // carriage.frontPosition();
          elevator.moveSmooth(ROCKET_LOW_HATCH);
        } else if (rightJoystick.getRisingEdge(6)) {
          // carriage.frontPosition();
          elevator.moveSmooth(ROCKET_MID_HATCH);
        }
      }

      // hatch intaking and placing
      if (rightJoystick.getRawButton(1)) {
        carriage.extendBCV();
        // carriage.stopDelay();
      } else {
        carriage.retractBCV();

        /*
         * if (carriage.getDelayState() == false) { carriage.openBCV();
         * carriage.startDelay(); } if (carriage.getDelayState() == true) {
         * carriage.incrementCounter(); } if (carriage.getDelayState() == true &&
         * carriage.getCount() == 10) { carriage.retractBCV(); carriage.stopDelay(); }
         */
      }

      if (rightJoystick.getRawButton(2)) {
        carriage.closeBCV();
      } else {
        carriage.openBCV();
      }

    }
    // cargo mode
    else {
      carriage.closeArms();
      carriage.closeBCV();
      carriage.retractBCV();

      /*
       * if(leftJoystick.getRisingEdge(2)) { carriage.backPosition(); }
       */

      // elevator setpoints
      // only allows driver to move elevator while the carriage is on the front
      if (carriage.getCurrentOrientation()) {
        if (rightJoystick.getRisingEdge(3)) {
          carriage.frontPosition();
          elevator.moveSmooth(2.0);
        } else if (rightJoystick.getRisingEdge(4)) {
          carriage.topCargoPosition(); // carriage needs to be tilted upwards
          elevator.moveSmooth(ROCKET_HIGH_CARGO);
        } else if (rightJoystick.getRisingEdge(5)) {
          carriage.frontPosition();
          elevator.moveSmooth(ROCKET_LOW_CARGO);
        } else if (rightJoystick.getRisingEdge(6)) {
          carriage.frontPosition();
          elevator.moveSmooth(ROCKET_MID_CARGO);
        } else if (leftJoystick.getRisingEdge(2)) {
          carriage.frontPosition();
          elevator.moveSmooth(35.5);
        }
      }

      // intaking cargo through cargo intake and carriage
      if (rightJoystick.getRawButton(1)) {
        cargoIntake.bottomPosition();
        cargoIntake.runIntake();
        carriage.spinArmWheels(1);
      }

      else {
        cargoIntake.stopIntake();
        carriage.spinArmWheels(0.05);
      }

      // spits ball out of carriage
      if (rightJoystick.getRawButton(2)) {
        carriage.spinArmWheels(-1);
      } else {
        // carriage.spinArmWheels(0.0);
      }

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
