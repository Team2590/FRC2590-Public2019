package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.settings.HatchIntakeSettings;
import frc.util.NemesisVictor;

/**
 * Grabs the hatch panel from the floor
 */
public class HatchIntake extends Subsystem implements RobotMap, HatchIntakeSettings {

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
    STOPPED, INTAKE, OUTTAKE
  }

  private Solenoid intakePiston;
  private NemesisVictor hatchIntakeMotor;

  public HatchIntake() {
    intakePiston = new Solenoid(INTAKE_SOLENOID);
    hatchIntakeMotor = new NemesisVictor(HATCH_INTAKE);
  }

  public void update() {
    switch (hatchState) {
    case STOPPED:
      hatchIntakeMotor.set(ControlMode.PercentOutput, 0.0);
      break;

    case INTAKE:
      hatchIntakeMotor.set(ControlMode.PercentOutput, 0.5);
      break;

    case OUTTAKE:
      hatchIntakeMotor.set(ControlMode.PercentOutput, -0.35);
      break;

    default:
      hatchIntakeMotor.set(ControlMode.PercentOutput, 0.0);
      break;
    }
  }

  public void stopIntake() {
    hatchState = States.STOPPED;
  }

  public void runIntake() {
    hatchState = States.INTAKE;
  }

  public void reverseIntake() {
    hatchState = States.OUTTAKE;
  }

  public void stow() {
    intakePiston.set(false);
  }

  public void drop() {
    intakePiston.set(true);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
