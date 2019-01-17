package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Grabs the hatch panel from the floor
 */
public class HatchIntake extends Subsystem implements RobotMap {

  // new singleton
  private static HatchIntake hatchInstance = null;

  public static HatchIntake getHatchIntakeInstance() {
    if (hatchInstance == null) {
      hatchInstance = new HatchIntake();
    }
    return hatchInstance;
  }

  // enum to control the states of the hatch intake
  private States hatchState = States.STOPPED;

  private enum States {
    STOPPED, INTAKE, OUTTAKE, STOWED, DROPPED
  }

  private Solenoid intakePiston;
  private CANSparkMax hatchIntakeMotor;

  // constructor
  public HatchIntake() {
    intakePiston = new Solenoid(intake_solenoid);
    // the hatch intake motor is connected to a 775, hence it is brushed
    hatchIntakeMotor = new CANSparkMax(hatch_intake, MotorType.kBrushed);
  }

  // called every loop of teleop periodic
  public void update() {
    switch (hatchState) {
    case STOPPED:
      hatchIntakeMotor.set(0.0);
      break;

    case INTAKE:
      hatchIntakeMotor.set(-1.0);
      break;

    case OUTTAKE:
      hatchIntakeMotor.set(1.0);
      break;

    case STOWED:
      intakePiston.set(false);
      break;

    case DROPPED:
      intakePiston.set(true);
      break;

    default:
      hatchIntakeMotor.set(0.0);
      System.out.println("Hatch Intake Default State");
      break;
    }
  }

  public void runIntake() {
    hatchState = States.INTAKE;
  }

  public void stopIntake() {
    hatchState = States.STOPPED;
  }

  public void reverseIntake() {
    hatchState = States.OUTTAKE;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
