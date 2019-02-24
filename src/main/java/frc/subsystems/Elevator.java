/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.controllers.ConstantCurrent;
import frc.controllers.MotionProfile;
import frc.robot.RobotMap;
import frc.settings.ElevatorSettings;
import frc.util.NemesisCANEncoder;

/**
 * Moves the Cargo and Hatch Panels to set heights to score points
 * 
 * @author Harsh Padhye, Rohan Bhatnagar
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

  private ConstantCurrent manualController;

  // setpoint to hold carriage in place after moving
  private double setpoint;

  private boolean manual;

  public Elevator() {
    elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR, MotorType.kBrushless);
    elevatorEncoder = new NemesisCANEncoder(elevatorMotor);
    failSafeEncoder = new NemesisCANEncoder(elevatorMotor);

    setConversionFactors();

    elevatorMotor.setIdleMode(IdleMode.kBrake);

    elevatorController = new MotionProfile(ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KV, ELEVATOR_KA, ELEVATOR_MAX_VEL,
        ELEVATOR_MAX_ACC, ELEVATOR_TOLERANCE, elevatorEncoder, elevatorMotor);

    manualController = new ConstantCurrent(elevatorMotor);

    setpoint = 0.0;

    manual = false;
  }

  // called every loop of teleop periodic
  public void update() {

    //System.out.println("elevator " + getHeight());
    switch (elevatorState) {
    case STOPPED:
      // proportional error calcualtion
      double power = (setpoint - getHeight()) * ELEVATOR_HOLD_CONSTANT;
      if (getHeight() < 5) {
        power = 0.0;
      }

      elevatorMotor.set(power);

      break;

    case MOVING:

      if (manual) {
        this.setpoint = getHeight();
        manualController.calculate();
        if (manualController.isDone()) {
          stopElevator();
        }
      } else {
        elevatorController.calculate();
        if (elevatorController.isDone()) {
          stopElevator();
        }
      }

      break;

    default:
      System.out.println("Elevator Default State");
      break;
    }

  }

  public void moveSmooth(double setpoint) {
    manual = false;
    elevatorController.setSetpoint(setpoint);
    this.setpoint = setpoint;
    elevatorState = States.MOVING;
  }

  public void moveManually(double speed) {
    manual = true;
    manualController.setSetpoint(speed);
    elevatorState = States.MOVING;
  }

  /**
   * Converts integrated encoders to the appropriate units of measurement
   */
  public void setConversionFactors() {
    // converts from rotations to distance (inches)
    // Turning shaft diameter is 1 in
    elevatorEncoder.setPositionConversionFactor(Math.PI / GEAR_RATIO);
    failSafeEncoder.setPositionConversionFactor(Math.PI / GEAR_RATIO);
    // converts from RPM to velocity (in/s)
    elevatorEncoder.setVelocityConversionFactor(Math.PI / (GEAR_RATIO * 60.0));
    failSafeEncoder.setVelocityConversionFactor(Math.PI / (GEAR_RATIO * 60.0));
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
