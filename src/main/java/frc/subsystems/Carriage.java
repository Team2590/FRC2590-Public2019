/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.settings.CarriageSettings;

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
    STOPPED, CARGO_MODE, HATCH_MODE
  }

  private Solenoid bcvPiston; // opens and closes the bicuspid valve
  private Solenoid armPiston; // opens and closes the intake arms

  private VictorSPX leftMotor;
  private VictorSPX rightMotor;

  public Carriage() {
    bcvPiston = new Solenoid(BCV_SOLENOID);
    armPiston = new Solenoid(ARM_SOLENOID);

    leftMotor = new VictorSPX(LEFT_CARRIAGE_MOTOR);
    rightMotor = new VictorSPX(RIGHT_CARRIAGE_MOTOR);
  }

  public void update() {
    switch (carriageState) {
    case STOPPED:
      setSpeeds(0, 0);
      break;

    case CARGO_MODE:
      closeArms();
      break;

    case HATCH_MODE:
      openArms();
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

  public void openBCV() {
    bcvPiston.set(true);
  }

  public void closeBCV() {
    bcvPiston.set(false);
  }

  public void openArms() {
    armPiston.set(false);
  }

  public void closeArms() {
    armPiston.set(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
