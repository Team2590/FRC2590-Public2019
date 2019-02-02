/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.controllers.MotionProfile;
import frc.robot.RobotMap;

/**
 * Moves the Cargo and Hatch Panels to set heights to score points
 * @author Harsh Padhye, Chinmay Savanur
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

  //Elevator Motor
  CANSparkMax elevatorMotor;

  //Elevator Encoder
  CANEncoder elevatorEncoder;

  Encoder temp;

  //Motion profile controller to move elevator smoothly and accurately
  MotionProfile elevatorController;


  public Elevator() {
    // the elevator motor is connected to a 775, hence it is brushed
    elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR, MotorType.kBrushed);
    elevatorEncoder = new CANEncoder(elevatorMotor);
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

  public void moveSmooth(double setpoint, double vel, double acc) {

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
