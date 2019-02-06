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
import frc.controllers.MotionProfile;
import frc.robot.RobotMap;
import frc.settings.ElevatorSettings;
import frc.util.NemesisCANEncoder;

/**
 * Moves the Cargo and Hatch Panels to set heights to score points
 * 
 * @author Harsh Padhye, Chinmay Savanur
 */
public class Elevator extends Subsystem implements RobotMap, ElevatorSettings {

  // new singleton
  private static Elevator elevatorInstance = null;

  public static Elevator getElevatorInstance() {
    if (elevatorInstance == null) {
      elevatorInstance = new Elevator();
    }
    return elevatorInstance;
  }

  // enum to control the state of the elevator
  private States elevatorState = States.STOPPED;

  private enum States {
    STOPPED, MOVING
  }

  // Elevator Motor
  private CANSparkMax elevatorMotor;

  // Elevator Encoder
  private NemesisCANEncoder elevatorEncoder;
  private NemesisCANEncoder failSafeEncoder;

  // Motion profile controller to move elevator smoothly and accurately
  private MotionProfile elevatorController;

  // setpoint to hold carriage in place after moving
  private double setpoint;

  public Elevator() {
    // the elevator motor is connected to a 775, hence it is brushed
    elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR, MotorType.kBrushed);
    elevatorEncoder = new NemesisCANEncoder(elevatorMotor);
    failSafeEncoder = new NemesisCANEncoder(elevatorMotor);

    elevatorController = new MotionProfile(ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KV, ELEVATOR_KA, ELEVATOR_MAX_VEL,
        ELEVATOR_MAX_ACC, ELEVATOR_TOLERANCE, elevatorEncoder, elevatorMotor);

    setpoint = 0.0;
  }

  // called every loop of teleop periodic
  public void update() {
    switch (elevatorState) {
    case STOPPED:

      double error = setpoint - getHeight();
      elevatorMotor.set(error * ELEVATOR_HOLD_CONSTANT);

      break;

    case MOVING:

      // calculates motor output and writes to the motor
      elevatorController.calculate();

      if (elevatorController.isDone()) {
        elevatorState = States.STOPPED;
      }

      break;

    default:
      System.out.println("Elevator Default State");
      break;
    }
  }

  public void moveSmooth(double setpoint) {
    elevatorController.setSetpoint(setpoint);
    this.setpoint = setpoint;
    elevatorState = States.MOVING;
  }

  public double getHeight() {
    return elevatorEncoder.getPosition();
  }

  public void stopElevator() {
    elevatorState = States.STOPPED;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
