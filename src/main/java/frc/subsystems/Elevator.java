/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Moves the Cargo and Hatch Panels to set heights to score points
 */
public class Elevator extends Subsystem implements RobotMap {

  // new singleton
  private static Elevator elevatorInstance = null;

  public static Elevator getElevatorInstance() {
    if (elevatorInstance == null) {
      elevatorInstance = new Elevator();
    }
    return elevatorInstance;
  }

  //enum to control the state of the elevator
  private States elevatorState = States.STOPPED;
  private enum States {
    STOPPED, MOVING
  }

  CANSparkMax elevatorMotor;

  // constructor
  public Elevator() {
    // the elevator motor is connected to a 775, hence it is brushed
    elevatorMotor = new CANSparkMax(elevator_motor, MotorType.kBrushed);
  }

  // called every loop of teleop periodic
  public void update() {
    switch(elevatorState){
      case STOPPED:
        elevatorMotor.set(0.0);
        break;

      case MOVING:
        break;

      default:
        System.out.println("Elevator Default State");
        break;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
