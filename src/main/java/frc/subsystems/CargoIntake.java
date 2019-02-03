/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.settings.CargoIntakeSettings;

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
    STOPPED, INTAKE, OUTTAKE
  }

  private VictorSPX cargoIntakeMotor;

  // constructor
  public CargoIntake() {
    // the cargo intake motor is connected to a 775, hence it is brushed
    cargoIntakeMotor = new VictorSPX(CARGO_INTAKE);
  }

  // called every loop of teleop periodic
  public void update() {
    switch (cargoState) {
    case STOPPED:
      cargoIntakeMotor.set(ControlMode.PercentOutput, 0.0);
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
