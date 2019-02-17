/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.controllers.MotionProfile;
import frc.robot.RobotMap;
import frc.settings.CarriageSettings;
import frc.util.NemesisVictor;

/**
 * Hatch and Cargo manipulating carraige on the elevator
 */
public class Carriage extends Subsystem implements RobotMap, CarriageSettings {

  private static Carriage carriageInstance = null;

  public static Carriage getCarriageInstance() {
    if (carriageInstance == null) {
      carriageInstance = new Carriage();
    }
    return carriageInstance;
  }

  private States carriageState = States.STOPPED;

  private enum States {
    STOPPED, MOVING, CARGO_MODE, HATCH_MODE
  }

  private Solenoid bcvFingers; // opens and closes the bicuspid valve
  private Solenoid bcvExtender; // extends and retracts the bicuspid valve
  private Solenoid armPiston; // opens and closes the intake arms

  private NemesisVictor leftMotor;
  private NemesisVictor rightMotor;
  private NemesisVictor swingMotor;

  private AnalogPotentiometer carriagePot;

  private MotionProfile carriageController;

  private int currentPosition;

  private boolean isOnFront;
  // if the carriage is backwards, and the driver wants to go to hatch mode, the
  // carriage will swing to the front, and insteading of stopping, will
  // automatically switch states to hatch mode
  private boolean moveToHatchMode;

  public Carriage() {
    bcvFingers = new Solenoid(BCV_FINGER_SOLENOID);
    bcvExtender = new Solenoid(BCV_EXTENDER_SOLENOID);
    armPiston = new Solenoid(ARM_SOLENOID);

    // leftMotor = new NemesisVictor(LEFT_CARRIAGE_MOTOR);
    // rightMotor = new NemesisVictor(RIGHT_CARRIAGE_MOTOR);
    // swingMotor = new NemesisVictor(SWING_CARRIAGE_MOTOR);

    // carriagePot = new AnalogPotentiometer(CARRIAGE_POTENTIOMETER);

    // carriageController = new MotionProfile(CARRIAGE_KP, CARRIAGE_KI, CARRIAGE_KV,
    // CARRIAGE_KA, CARRIAGE_MAX_VEL,
    // CARRIAGE_MAX_ACC, CARRIAGE_TOLERANCE, carriageAccelerometer, swingMotor);

    currentPosition = 0;
    // isOnFront = false; // starts backwards in frame perimeter
    // moveToHatchMode = false;

  }

  public void update() {
    switch (carriageState) {
    case STOPPED:
      setSpeeds(0, 0);
      swingMotor.set(ControlMode.PercentOutput, 0.0);
      break;

    case MOVING:
      // makes sure the elements of the carraige are stowed before moving
      closeArms();
      closeBCV();
      retractBCV();

      // moves the carriage to desired setpoint via motion profiling
      carriageController.calculate();

      // checks if the controller is done
      // and which mode to engage (hatch, cargo, stopped)
      if (carriageController.isDone()) {
        if (moveToHatchMode) {
          carriageState = States.HATCH_MODE;
        } else {
          carriageState = States.STOPPED;
        }
      }
      break;

    case CARGO_MODE:
      retractBCV(); // stows away hatch manipulation components
      closeBCV();
      closeArms(); // closes intake arms, allows carriage to swing back and forth through elevator
      break;

    case HATCH_MODE:
      // a button click engages hatch mode
      // the following logic takes determines if the carriage is currently in the
      // front or the back and moves the carriage/opens the arms accordingly
      // carriage cannot swing through elevator with open arms
      if (getCurrentOrientation() == 1) {
        // opens the arms to make space for hatch panel
        openArms();
        moveToHatchMode = false;
      } else {
        // moves carriage to front, automatically engages hatch mode afterwards
        moveToHatchMode = true;
        swingCarriage(1);
      }
      break;

    default:
      System.out.println("Carriage default state");
      break;
    }

  }

  /**
   * sets the speeds for the arm motors at the same time
   * 
   * @param left  drive speed for the left motor controller
   * @param right drive speed for the right motor controller
   */
  public void setSpeeds(double left, double right) {
    leftMotor.set(ControlMode.PercentOutput, left);
    rightMotor.set(ControlMode.PercentOutput, right);
  }

  /**
   * Moves the carriage back and forth through the elevator
   * 
   * @param position 0 : Back Position of carriage
   * @param position 1 : Front Position of carriage
   * @param position 2 : Hatch handoff position of carriage
   */
  public void swingCarriage(int position) {
    if (position == 2) {
      carriageController.setSetpoint(HATCH_HANDOFF_POSITION);
      currentPosition = 2;
    } else if (position == 1) {
      carriageController.setSetpoint(FRONT_POSITION);
      currentPosition = 1;
    } else {
      carriageController.setSetpoint(BACK_POSITION);
      currentPosition = 0; 
    }
    carriageState = States.MOVING;
  }

  /**
   * Opens the BCV finger to grab the hatch
   * Can only extend when arms are open
   */
  public void openBCV() {
    bcvFingers.set(true);
  }

  /**
   * Closes BCV fingers to release hatch
   */
  public void closeBCV() {
    bcvFingers.set(false);
  }

  /**
   * Extends the slide outwards
   */
  public void extendBCV() {
    bcvExtender.set(true);
  }

  /**
   * Retracts the slide back in
   */
  public void retractBCV() {
    bcvExtender.set(false);
  }

  /**
   * Opens the intake arms to create space for the hatch
   */
  public void openArms() {
    armPiston.set(false);
  }

  /**
   * Closes intake arms to grip the cargo
   */
  public void closeArms() {
    armPiston.set(true);
  }

  /**
   * Calculates whether the carriage is on the front side of the elevator based on
   * potentiometer values
   * 
   * @return true if the carriage is on the front side
   */
  public int getCurrentOrientation() {
    int orientation = 1;
    // do the pot calculation here
    isOnFront = true; // subject tto change based on calcs
    return orientation;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
