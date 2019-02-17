/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.controllers.MotionProfile;
import frc.robot.RobotMap;
import frc.settings.CargoIntakeSettings;
import frc.util.NemesisVictor;

/**
 * Intakes the cargo (playground balls) from the floor
 */
public class CargoIntake extends Subsystem implements RobotMap, CargoIntakeSettings {

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
    STOPPED, MOVING, INTAKE, OUTTAKE
  }

  private NemesisVictor cargoIntakeMotor;
  private NemesisVictor cargoArticulateMotor;
  private AnalogPotentiometer cargoPot;
  private MotionProfile cargoArticulateController;

  public CargoIntake() {
    cargoIntakeMotor = new NemesisVictor(CARGO_INTAKE);
    // cargoArticulateMotor = new NemesisVictor(CARGO_ARTICULATOR);
    // cargoPot = new AnalogPotentiometer(CARGO_POTENTIOMETER);
    // cargoArticulateController = new MotionProfile(CARGO_INTAKE_KP,
    // CARGO_INTAKE_KI,
    // CARGO_INTAKE_KV, CARGO_INTAKE_KA,
    // CARGO_INTAKE_MAX_VEL, CARGO_INTAKE_MAX_ACC, CARGO_INTAKE_TOLERANCE, cargoPot,
    // cargoArticulateMotor);
  }

  public void update() {
    switch (cargoState) {
    case STOPPED:
      cargoIntakeMotor.set(ControlMode.PercentOutput, 0.0);
      break;

    case MOVING:
      //stops intake wheels from spinning while the arm is moving
      cargoIntakeMotor.set(ControlMode.PercentOutput, 0.0);

      //moves arm via motion profiling
      cargoArticulateController.calculate();
      if (cargoArticulateController.isDone()) {
        cargoState = States.STOPPED;
      }
      break;

    case INTAKE:
      cargoIntakeMotor.set(ControlMode.PercentOutput, 1.0);
      break;

    case OUTTAKE:
      cargoIntakeMotor.set(ControlMode.PercentOutput, -1.0);
      break;

    default:
      cargoIntakeMotor.set(ControlMode.PercentOutput, 0.0);
      System.out.println("Cargo Intake Default State");
      break;
    }
  }

  public void runIntake() {
    cargoState = States.INTAKE;
  }

  public void stopIntake() {
    cargoState = States.STOPPED;
  }

  public void reverseIntake() {
    cargoState = States.OUTTAKE;
  }

  public void moveCargoIntake(double setpoint) {
    cargoArticulateController.setSetpoint(setpoint);
    cargoState = States.MOVING;
  }

  public double getPosition() {
    return cargoPot.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
