/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.controllers.MotionProfile;
import frc.robot.RobotMap;
import frc.settings.CarriageSettings;
import frc.settings.FieldSettings;
import frc.util.NemesisPotentiometer;
import frc.util.NemesisVictor;

/**
 * Hatch and Cargo manipulating carraige on the elevator
 * 
 * @author Harsh Padhye, Ishan Arora
 */
public class Carriage extends Subsystem implements RobotMap, CarriageSettings, FieldSettings {

  private static Carriage carriageInstance = null;

  public static Carriage getCarriageInstance() {
    if (carriageInstance == null) {
      carriageInstance = new Carriage();
    }
    return carriageInstance;
  }

  private States carriageState = States.STOPPED;

  private enum States {
    STOPPED, MOVING
  }

  private Solenoid bcvFingers; // opens and closes the bicuspid valve
  private Solenoid bcvExtender; // extends and retracts the bicuspid valve
  private Solenoid armPiston; // opens and closes the intake arms

  private NemesisVictor leftMotor;
  private NemesisVictor rightMotor;
  private NemesisVictor swingMotor;

  private NemesisPotentiometer carriagePot;

  private MotionProfile carriageController;

  private double setpoint;
  private double errorSum;
  private double lastError;

  private boolean isInDelay;
  private double counter;

  public Carriage() {
    bcvFingers = new Solenoid(BCV_FINGER_SOLENOID);
    bcvExtender = new Solenoid(BCV_EXTENDER_SOLENOID);
    armPiston = new Solenoid(ARM_SOLENOID);

    leftMotor = new NemesisVictor(LEFT_CARRIAGE_MOTOR);
    rightMotor = new NemesisVictor(RIGHT_CARRIAGE_MOTOR);
    swingMotor = new NemesisVictor(SWING_CARRIAGE_MOTOR);

    rightMotor.setInverted(true);
    swingMotor.setInverted(true);

    carriagePot = new NemesisPotentiometer(CARRIAGE_POTENTIOMETER, 360.0);
    carriagePot.setSlipLimit(270.0);
    carriagePot.setName("Carriage Potentiometer");

    carriageController = new MotionProfile(CARRIAGE_KP, CARRIAGE_KI, CARRIAGE_KV, CARRIAGE_KA, CARRIAGE_MAX_VEL,
        CARRIAGE_MAX_ACC, CARRIAGE_TOLERANCE, carriagePot, swingMotor);

    setpoint = getAngle(); // should be set to getAngle(), set to 180 for testing purposes

    errorSum = 0.0;
    lastError = 0.0;

    isInDelay = false;
  }

  public void update() {

    switch (carriageState) {

    case STOPPED:
      carriageController.endProfile();
      double error = setpoint - getAngle();
      double deltaError = error - lastError;
      double command = 0.0;

      if (setpoint > 100 && setpoint < 170) {
        // carriage is help in place in the from position, requires PID controller
        errorSum += error * REFRESH_RATE;
        command = error * kP_HOLD_CONSTANT + errorSum * kI_HOLD_CONSTANT + deltaError * kD_HOLD_CONSTANT;
        lastError = error;

      } else if (setpoint < 80 || setpoint > 170) {
        // turns the controller off when on the hard stop
        command = 0.0;
        errorSum = 0.0;
        lastError = 0.0;

      } else {
        // the setpoint is around 90 deg, so the carriage acts as an inverted pendulum
        // only requires P and D gains to stabilize
        errorSum = error * REFRESH_RATE;
        command = error * kP_HOLD_CONSTANT + errorSum * kI_HOLD_CONSTANT + deltaError * kD_HOLD_CONSTANT;
        lastError = error;
      }

      swingMotor.set(ControlMode.PercentOutput, command);
      break;

    case MOVING:
      //solenoids are automatically stowed in the Robot.java class, since constant
      //back and forth firing will drop game elements

      setSpeeds(0.0, 0.0);

      // moves the carriage to desired setpoint via motion profiling
      carriageController.calculate();

      // checks if the controller is done
      if (carriageController.isDone()) {
        carriageState = States.STOPPED;
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
   * @param setpoint the desired position of the carriage
   */
  public void swingCarriage(double setpoint) {
    carriageController.setSetpoint(setpoint);
    this.setpoint = setpoint;
    this.errorSum = 0.0;
    this.lastError = 0.0;
    carriageState = States.MOVING;
  }

  /**
   * Flips the carriage to the back position
   */

  public void backPosition() {
    swingCarriage(BACK_POSITION);
  }

  /**
   * Flips the carriage to the front position
   */
  public void frontPosition() {
    if (getCurrentOrientation() == false) {
      swingCarriage(FRONT_POSITION);
    }
  }

  /**
   * Flips the carriage the hatch-handoff position This is tilted downwards from
   * the front position
   */
  public void hatchHandoffPosition() {
    swingCarriage(HATCH_HANDOFF_POSITION);
  }

  /**
   * Flips the carriage to the upright position The carriage acts as an inverted
   * pendulum
   */
  public void uprightPosition() {
    swingCarriage(UPRIGHT_POSITION);
  }

  /**
   * Flips the carriage to the top cargo position
   */
  public void topCargoPosition() {
    swingCarriage(TOP_CARGO_POSITION);
  }

  /**
   * spins both carriage intake wheels
   * 
   * @param speed The power at which to run the motors [-1,1]
   */
  public void spinArmWheels(double speed) {
    setSpeeds(speed, speed);
  }

  /**
   * Moves the carriage manually
   * 
   * @param speed The power at which to move the carriage [-1,1]
   */
  public void manualSwing(double speed) {
    swingMotor.set(ControlMode.PercentOutput, speed);
    setpoint = getAngle();
  }

  /**
   * Use to check if it is safe to open the arms
   * 
   * @return true if the carriage is on the front, false if at the back
   */
  public boolean getCurrentOrientation() {
    return carriagePot.get() > 100.0;
  }

  public boolean getDelayState() {
    return isInDelay;
  }

  public double getCount() {
    return counter;
  }

  public boolean isMoving() {
    return carriageState == States.MOVING;
  }

  public void startDelay() {
    counter = 0;
    isInDelay = true;
  }

  public void incrementCounter() {
    counter += 1;
  }

  public void stopDelay() {
    isInDelay = false;
    counter = 0;
  }

  /**
   * Opens the BCV finger to grab the hatch Can only extend when arms are open
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

  public void toggleBCV() {
    boolean curr = bcvFingers.get();
    bcvFingers.set(!curr);
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
    armPiston.set(true);
  }

  /**
   * Closes intake arms to grip the cargo
   */
  public void closeArms() {
    armPiston.set(false);
  }

  public void holdPosition() {
    carriageController.endProfile();
    carriageState = States.STOPPED;
  }

  /**
   * @return The current angle of the carriage (back 0, front 180)
   */
  public double getAngle() {
    return carriagePot.get();
  }

  /**
   * REMOVE THIS METHOD AFTER TUNING IS FINISHED
   * 
   * @param setpoint
   */
  public void setCarriageSetpoint(double setpoint) {
    this.setpoint = setpoint;
    errorSum = 0.0;
    lastError = 0.0;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
