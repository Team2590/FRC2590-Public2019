/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.controllers.ConstantCurrent;
import frc.controllers.MotionProfile;
import frc.robot.RobotMap;
import frc.settings.CargoIntakeSettings;
import frc.settings.FieldSettings;
import frc.util.NemesisVictor;

/**
 * Intakes the cargo (playground balls) from the floor
 * 
 * @author Harsh Padhye, Aditya Ganesh
 */
public class CargoIntake extends Subsystem implements RobotMap, CargoIntakeSettings, FieldSettings {

  // new singleton
  private static CargoIntake cargoInstance = null;

  public static CargoIntake getCargoIntakeInstance() {
    if (cargoInstance == null) {
      cargoInstance = new CargoIntake();
    }
    return cargoInstance;
  }

  private States cargoState = States.STOPPED;

  private enum States {
    STOPPED, MOVING
  }

  private NemesisVictor cargoIntakeMotor;
  private NemesisVictor cargoArticulateMotor;
  private AnalogPotentiometer cargoPot;

  private MotionProfile cargoArticulateController;
  private ConstantCurrent manualController;

  private boolean manual;

  private double setpoint;
  private double errorSum;
  private double lastError;

  public CargoIntake() {
    cargoIntakeMotor = new NemesisVictor(CARGO_INTAKE);
    cargoArticulateMotor = new NemesisVictor(CARGO_ARTICULATOR);
    cargoPot = new AnalogPotentiometer(CARGO_POTENTIOMETER, 360.0);

    cargoArticulateMotor.setInverted(true);

    cargoIntakeMotor.setNeutralMode(NeutralMode.Brake);
    cargoArticulateMotor.setNeutralMode(NeutralMode.Brake);

    cargoArticulateController = new MotionProfile(CARGO_INTAKE_KP, CARGO_INTAKE_KI, CARGO_INTAKE_KV, CARGO_INTAKE_KA,
        CARGO_INTAKE_MAX_VEL, CARGO_INTAKE_MAX_ACC, CARGO_INTAKE_TOLERANCE, cargoPot, cargoArticulateMotor);

    manualController = new ConstantCurrent(cargoArticulateMotor);

    manual = false;

    setpoint = getAngle();

    errorSum = 0.0;
    lastError = 0.0;
  }

  public void update() {

    switch (cargoState) {
    case STOPPED:
      double error = setpoint - getAngle();
      double command = 0.0;
      errorSum += error * REFRESH_RATE;
      double deltaError = error - lastError;
      // adds the error terms
      command = error * kP_HOLD_CONSTANT + errorSum * kI_HOLD_CONSTANT + deltaError * kD_HOLD_CONSTANT;

      lastError = error;

      cargoArticulateMotor.set(ControlMode.PercentOutput, command);
      // System.out.println("Command :: " + command);
      break;

    case MOVING:

    if(manual) {
      this.setpoint = getAngle();
      manualController.calculate();
      if(manualController.isDone()) {
        holdPosition();
      }
    } else {
        cargoArticulateController.calculate();
        if (cargoArticulateController.isDone()) {
          holdPosition();
        }
    }

      break;

    default:
      System.out.println("Cargo Intake Default State");
      break;
    }
  }

  public void runIntake() {
    cargoIntakeMotor.set(ControlMode.PercentOutput, 0.8);
  }

  public void stopIntake() {
    cargoIntakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void reverseIntake() {
    cargoIntakeMotor.set(ControlMode.PercentOutput, -1.0);
  }

  public void moveManually(double speed) {
    manual = true;
    manualController.setSetpoint(speed);
    cargoState = States.MOVING;
  }

  public void moveCargoIntake(double setpoint) {
    manual = false;
    cargoArticulateController.setSetpoint(setpoint);
    this.setpoint = setpoint;
    cargoState = States.MOVING;
  }

  public void bottomPosition() {

    moveCargoIntake(BOTTOM_POSITION);
  }

  public void topPosition() {
    moveCargoIntake(TOP_POSITION);
  }

  public void holdPosition() {
    cargoState = States.STOPPED;
  }

  public double getAngle() {
    return cargoPot.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
