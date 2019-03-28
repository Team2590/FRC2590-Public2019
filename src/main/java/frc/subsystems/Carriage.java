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

  private NemesisVictor intakeMotor;
  private NemesisVictor swingMotor;

  private NemesisPotentiometer carriagePot;

  private MotionProfile carriageController;

  private double setpoint;
  private double errorSum;
  private double lastError;

  private boolean isInDelay;
  private boolean cutPower;
  private int delayCounter;

  public Carriage() {
    bcvFingers = new Solenoid(BCV_FINGER_SOLENOID);

    intakeMotor = new NemesisVictor(INTAKE_CARRIAGE_MOTOR);
    swingMotor = new NemesisVictor(SWING_CARRIAGE_MOTOR, 6, true, 5.0, 10);

    intakeMotor.setInverted(false);
    swingMotor.setInverted(false);

    carriagePot = new NemesisPotentiometer(CARRIAGE_POTENTIOMETER, 360.0);
    carriagePot.setSlipLimit(270.0);

    carriageController = new MotionProfile(CARRIAGE_KP, CARRIAGE_KI, CARRIAGE_KV, CARRIAGE_KA, CARRIAGE_MAX_VEL,
        CARRIAGE_MAX_ACC, CARRIAGE_TOLERANCE, carriagePot, swingMotor);

    setpoint = getAngle(); // should be set to getAngle(), set to 180 for testing purposes

    errorSum = 0.0;
    lastError = 0.0;

    isInDelay = false;
    cutPower = false;
    delayCounter = 0;
  }

  public void update() {
    switch (carriageState) {

    case STOPPED:
      carriageController.endProfile();
      double error = setpoint - getAngle();
      double deltaError = error - lastError;
      double command = 0.0;

      if (setpoint > 30 && setpoint < 140) { // 30 < setpoint < 140
        // carriage is held in place, requires PID controller
        errorSum += error * REFRESH_RATE;
        command = error * kP_HOLD_CONSTANT + errorSum * kI_HOLD_CONSTANT + deltaError * kD_HOLD_CONSTANT;
        lastError = error;

        //if power is cut, sets command to 0.0 regardless of setpoint
        if(cutPower) command = 0.0;

      } else {
        // turns the controller off when on the hard stop
        command = 0.0;
        errorSum = 0.0;
        lastError = 0.0;
      }

      swingMotor.pidWrite(command);
      break;

    case MOVING:

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
      swingCarriage(FRONT_POSITION);
  }

  /**
   * Flips the carriage to the latch position on the elevator
   */
  public void latchPosition() {
    swingCarriage(LATCH_POSITION);
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
   * spins carriage intake wheels
   * 
   * @param speed The power at which to run the motor [-1,1]
   */
  public void runIntake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
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

  public int getDelayCount() {
    return delayCounter;
  }

  public void startDelay() {
    delayCounter = 0;
    isInDelay = true;
  }

  public void stopDelay() {
    delayCounter = 0;
    isInDelay = false;
  }

  public void incrementCounter() {
    delayCounter++;
  }

  /**
   * Opens the BCV finger to grab the hatch Can only extend when arms are open
   */
  public void openBCV() {
    bcvFingers.set(false);
  }

  /**
   * Closes BCV fingers to release hatch
   */
  public void closeBCV() {
    bcvFingers.set(true);
  }

  public void toggleBCV() {
    boolean curr = bcvFingers.get();
    bcvFingers.set(!curr);
  }

  public void holdPosition() {
    carriageController.endProfile();
    carriageState = States.STOPPED;
  }

  public void stopHoldConstant() {
    cutPower = true;
  }

  /**
   * @return The current angle of the carriage (back 0, front 180)
   */
  public double getAngle() {
    return carriagePot.get();
  }

  public NemesisPotentiometer getPotentiometer() {
    return carriagePot;
  }

  @Override
  public void initDefaultCommand() {
  }
}
