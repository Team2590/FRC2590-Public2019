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
import frc.subsystems.Carriage;
import frc.subsystems.Climber;
import frc.subsystems.Drivetrain;
import frc.subsystems.Elevator;
import frc.util.Delayer;
import frc.util.Limelight;
import frc.util.NemesisJoystick;

public class Robot extends TimedRobot implements FieldSettings, ButtonMap {

  // joysticks
  private NemesisJoystick leftJoystick;
  private NemesisJoystick rightJoystick;
  private NemesisJoystick operatorJoystick;

  // susbsystems
  private static Drivetrain drivetrain;
  private static Carriage carriage;
  private static Elevator elevator;
  private static Climber climber;

  // PDP for current draw purposes
  private PowerDistributionPanel pdp;

  // Limelight camera for vision targeting
  private static Limelight limelight;

  private CameraServer camServer;

  // Compressor
  private Compressor compressor;

  // Looper holster for multithreaded control of subsystems
  private Looper enabledLooper;

  // Separate Delayers for subsystem functions
  private Delayer intakeCurrentDelay;
  private Delayer elevatorLatchDelay;
  private Delayer carriageLatchDelay;

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
    carriage = Carriage.getCarriageInstance();
    elevator = Elevator.getElevatorInstance();
    climber = Climber.getClimberInstance();

    pdp = new PowerDistributionPanel();

    // limelight camera
    limelight = Limelight.getLimelightInstance();

    enabledLooper = new Looper(REFRESH_RATE);

    intakeCurrentDelay = new Delayer();
    elevatorLatchDelay = new Delayer();
    carriageLatchDelay = new Delayer();

    // add subsystems to the Looper holster
    enabledLooper.register(drivetrain::update);
    //enabledLooper.register(carriage::update);
    enabledLooper.register(elevator::update);
    enabledLooper.register(climber::update);

    // instantiates compressor
    compressor = new Compressor();
    compressor.clearAllPCMStickyFaults();

    // default mode is cargo mode
    hatchButtonMode = false;
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
    carriage.holdPosition();
  }

  /**
   * Autonomous Mode, Robot moves on its own
   */
  @Override
  public void autonomousPeriodic() {

    limelight.update();

    // if (drivetrain.isTurnDone()) {
    drivetrain.teleopDrive(-leftJoystick.getYBanded(), rightJoystick.getXBanded());
    // }

    if (drivetrain.isHighGear()) {
      limelight.turnLimelightOff();
    } else {
      limelight.turnLimelightOn();
    }

    // manual shifting
    if (leftJoystick.getRisingEdge(UPSHIFT)) {
      drivetrain.manualGearShift(true);
    } else if (leftJoystick.getRisingEdge(DOWNSHIFT)) {
      drivetrain.manualGearShift(false);
    }

    /*
     * // Auto Align // input the limelight reading once to avoid latency issues if
     * (leftJoystick.getRisingEdge(AUTO_ALIGN)) {
     * drivetrain.turn(drivetrain.getHeading() +
     * limelight.getHorizontalAngleToTarget()); }
     */

    // McKillip Drive
    if (leftJoystick.getRawButton(AUTO_ALIGN)) {
      drivetrain.guideSteering(-leftJoystick.getYBanded(), limelight.getHorizontalAngleToTarget(),
          limelight.getCamTranYaw(), limelight.getCamTranZ(), limelight.getCamTranX());
    }

    if (rightJoystick.getPOV() == 0) {
      elevator.moveManually(0.5);
    } else if (rightJoystick.getPOV() == 180) {
      elevator.moveManually(-0.25);
    }

    // Operator has the ability to move the carriage
    if (operatorJoystick.getRisingEdge(CARRIAGE_FRONT)) {
      carriage.frontPosition();
    } else if (operatorJoystick.getRisingEdge(CARRIAGE_MIDDLE)) {
      carriage.uprightPosition();
    } else if (operatorJoystick.getRisingEdge(CARRIAGE_BACK)) {
      carriage.backPosition();
    }

    if (operatorJoystick.getRisingEdge(FORCE_TELEOP)) {
      drivetrain.forceTeleop();
    }

    // Switches between Hatch Mode and Ball Mode
    if (leftJoystick.getRisingEdge(HATCH_MODE)) {
      hatchButtonMode = true;
    } else if (leftJoystick.getRisingEdge(BALL_MODE)) {
      hatchButtonMode = false;
    }

    /**
     * Here lies the logic for button mapping between hatch and cargo mode Some
     * buttons serve homologous functions between the two modes (eg: elevator
     * setpoint buttons)
     */

    // hatch mode
    if (hatchButtonMode) {

      carriage.runIntake(0.0);

      // elevator setpoints
      // only allows driver to raise elevator while it is in the front position
      if (carriage.getCurrentOrientation()) {
        if (rightJoystick.getRisingEdge(ELEVATOR_GROUND)) {
          carriage.frontPosition();
          elevator.moveSmooth(1.0);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_HIGH)) {
          elevator.moveSmooth(ROCKET_HIGH_HATCH);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_LOW)) {
          elevator.moveSmooth(ROCKET_LOW_HATCH);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_MID)) {
          elevator.moveSmooth(ROCKET_MID_HATCH);

        }
      }

      if (rightJoystick.getRawButton(CLOSE_BCV)) {
        carriage.closeBCV();
      } else {
        carriage.openBCV();
      }

    }

    // cargo mode
    else {

      carriage.openBCV();

      // elevator setpoints
      // only allows driver to move elevator while the carriage is on the front
      if (carriage.getCurrentOrientation()) {
        if (rightJoystick.getRisingEdge(ELEVATOR_GROUND)) {
          carriage.frontPosition();
          elevator.moveSmooth(1.0);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_HIGH)) {
          carriage.topCargoPosition();
          elevator.moveSmooth(ROCKET_HIGH_CARGO);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_LOW)) {
          carriage.topCargoPosition();
          elevator.moveSmooth(ROCKET_LOW_CARGO);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_MID)) {
          carriage.topCargoPosition();
          elevator.moveSmooth(ROCKET_MID_CARGO);
        }
      }

      if (rightJoystick.getRawButton(BALL_INTAKE)) {
        carriage.runIntake(1.0);
      } else if (rightJoystick.getRawButton(BALL_OUTTAKE)) {
        carriage.runIntake(-1.0);
      } else {
        // nominal current to retain ball
        carriage.runIntake(0.2);
      }

      if (operatorJoystick.getRisingEdge(ELEVATOR_CARGO_SHIP)) {
        elevator.moveSmooth(ROCKET_MID_CARGO);
      }

    }
  }

  /**
   * Initialization of teleoperated mode, Switches from auto to teleop
   */
  @Override
  public void teleopInit() {
    enabledLooper.startLoops();
    drivetrain.resetAllSensors();
    carriage.holdPosition();
  }

  /**
   * Teleoperated Mode, Driver controls robot
   */
  @Override
  public void teleopPeriodic() {
    limelight.update();

    // System.out.println("Carriage " + carriage.getAngle());
    // System.out.println("Elev " + elevator.getHeight());
    // System.out.println("climber " + climber.getAngle());

    // This logic checks if the robot is still turning, to avoid switching states to
    // teleop in the middle of the control loop
    // Also prevents driver from moving the robot while it auto aligns

    if (drivetrain.isTurnDone()) {
      drivetrain.teleopDrive(-leftJoystick.getYBanded(), rightJoystick.getXBanded());
    }

    if (operatorJoystick.getRisingEdge(FORCE_TELEOP) || operatorJoystick.getRisingEdge(FORCE_TELEOP_FAILSAFE)) {
      drivetrain.forceTeleop();
    }

    if (drivetrain.isHighGear()) {
      limelight.turnLimelightOff();
    } else {
      limelight.turnLimelightOn();
    }

    // manual shifting
    if (leftJoystick.getRisingEdge(UPSHIFT)) {
      // limelight.turnLimelightOff();
      drivetrain.manualGearShift(true);
    } else if (leftJoystick.getRisingEdge(DOWNSHIFT)) {
      // limelight.turnLimelightOn();
      drivetrain.manualGearShift(false);
    }

    // McKillip Drive
    if (leftJoystick.getRawButton(AUTO_ALIGN)) {
      drivetrain.guideSteering(-leftJoystick.getYBanded(), limelight.getHorizontalAngleToTarget(),
          limelight.getCamTranYaw(), limelight.getCamTranZ(), limelight.getCamTranX());
    }

    // moving the elevator manually
    if (rightJoystick.getPOV() == 0) {
      elevator.moveManually(0.5);
    } else if (rightJoystick.getPOV() == 180) {
      elevator.moveManually(-0.5);
    }

    // Operator has the ability to move the carriage
    if (operatorJoystick.getRisingEdge(CARRIAGE_FRONT)) {
      carriage.frontPosition();
    } else if (operatorJoystick.getRisingEdge(CARRIAGE_MIDDLE)) {
      carriage.uprightPosition();
    } else if (operatorJoystick.getRisingEdge(CARRIAGE_BACK)) {
      carriage.backPosition();
    }

    // moving the climber arm manually
    if (operatorJoystick.getPOV() == 0) {
      climber.moveManually(0.5);
    } else if (operatorJoystick.getPOV() == 180) {
      climber.moveManually(-0.25);
    }

    if (operatorJoystick.getRisingEdge(CARRIAGE_LATCH)) {
      hatchButtonMode = true;
      carriage.uprightPosition();
      elevatorLatchDelay.startDelay(1.5);
      carriageLatchDelay.startDelay(3.0);
    }

    if (elevatorLatchDelay.checkDelayStatus()) {
      elevator.moveSmooth(LATCH_HEIGHT);
    }

    if (carriageLatchDelay.checkDelayStatus()) {
      carriage.latchPosition();
      carriage.stopHoldConstant();
    }

    // moves carriage and elevator to latch position, waiting to engage
    // if (operatorJoystick.getRisingEdge(CARRIAGE_LATCH)) {
    // hatchButtonMode = true;
    // carriage.uprightPosition();
    // elevator.startDelay();
    // carriage.startDelay();
    // } else {
    // // if the elevator has iterated 75 cycles (1.5 sec) in delay
    // if (elevator.getDelayCount() > 75 && elevator.getDelayState()) {
    // elevator.stopDelay();
    // elevator.moveSmooth(LATCH_HEIGHT);
    // } else if (elevator.getDelayState()) {
    // elevator.incrementCounter();
    // }

    // // delay for the carriage engaging the lock (3 seconds from button press)
    // if (carriage.getDelayCount() > 150 && carriage.getDelayState()) {
    // carriage.stopDelay();
    // carriage.latchPosition();
    // carriage.stopHoldConstant(); // prevents motor from applying power when
    // latched
    // } else if (carriage.getDelayState()) {
    // carriage.incrementCounter();
    // }

    // }

    if (operatorJoystick.getRisingEdge(ELEVATOR_HARDSTOP_DISABLE)) {
      elevator.disableHardstop();
    }

    // runs the elevator and climber arm downwards simultaneously
    if (operatorJoystick.getRawButton(LEVEL_3_CLIMBER_AND_ELEVATOR)) {
      elevator.moveManually(-0.6);
      climber.moveManually(0.5);
    } else if (operatorJoystick.getRawButton(LEVEL_2_CLIMBER_AND_ELEVATOR)) {
      elevator.moveManually(-0.3);
      climber.moveManually(0.25);
    }

    // runs the intake wheels on the climber arm
    if (operatorJoystick.getRawButton(CLIMBER_INTAKE)) {
      climber.intakeClimber();
    } else {
      climber.stopIntake();
    }

    // Switches between Hatch Mode and Ball Mode
    if (leftJoystick.getRisingEdge(HATCH_MODE)) {
      hatchButtonMode = true;
    } else if (leftJoystick.getRisingEdge(BALL_MODE)) {
      hatchButtonMode = false;
    }

    /**
     * Here lies the logic for button mapping between hatch and cargo mode Some
     * buttons serve homologous functions between the two modes (eg: elevator
     * setpoint buttons)
     */

    // hatch mode
    if (hatchButtonMode) {

      carriage.runIntake(0.0);

      // elevator setpoints
      // only allows driver to raise elevator while it is in the front position
      if (carriage.getCurrentOrientation()) {
        if (rightJoystick.getRisingEdge(ELEVATOR_GROUND)) {
          carriage.frontPosition();
          elevator.moveSmooth(1.0);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_HIGH)) {
          elevator.moveSmooth(ROCKET_HIGH_HATCH);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_LOW)) {
          elevator.moveSmooth(ROCKET_LOW_HATCH);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_MID)) {
          elevator.moveSmooth(ROCKET_MID_HATCH);

        }
      }

      if (rightJoystick.getRawButton(CLOSE_BCV)) {
        // limelight.turnLimelightOn();
        carriage.closeBCV();
      } else {
        // limelight.turnLimelightOff();
        carriage.openBCV();
      }

    }

    // cargo mode
    else {

      carriage.openBCV();

      // elevator setpoints
      // only allows driver to move elevator while the carriage is on the front
      if (carriage.getCurrentOrientation()) {

        if (rightJoystick.getRisingEdge(ELEVATOR_GROUND)) {
          carriage.frontPosition();
          elevator.moveSmooth(1.0);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_HIGH)) {
          carriage.topCargoPosition();
          elevator.moveSmooth(ROCKET_HIGH_CARGO);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_LOW)) {
          carriage.topCargoPosition();
          elevator.moveSmooth(ROCKET_LOW_CARGO);

        } else if (rightJoystick.getRisingEdge(ELEVATOR_MID)) {
          carriage.topCargoPosition();
          elevator.moveSmooth(ROCKET_MID_CARGO);
        }
      }

      if (rightJoystick.getRawButton(BALL_INTAKE)) {
        carriage.runIntake(1.0);
      } else if (rightJoystick.getRawButton(BALL_OUTTAKE)) {
        carriage.runIntake(-1.0);
      } else {
        // nominal current to retain ball
        carriage.runIntake(0.2);

      }

      if (rightJoystick.getFallingEdge(BALL_INTAKE)) {
        intakeCurrentDelay.startDelay(0.5);
      }

      if (intakeCurrentDelay.checkDelayStatus()) {
        if (carriage.getCurrentOrientation() && elevator.isGrounded() && carriage.hasCargo()) {
          elevator.moveSmooth(5.0);
        }
      }

      if (operatorJoystick.getRisingEdge(ELEVATOR_CARGO_SHIP)) {
        elevator.moveSmooth(ROCKET_MID_CARGO);
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

  public static Carriage getCarriageInstance() {
    return carriage;
  }

  public static Elevator getElevatorInstance() {
    return elevator;
  }

  public static Climber getClimberInstance() {
    return climber;
  }

  public static Limelight getLimelightInstance() {
    return limelight;
  }

}
