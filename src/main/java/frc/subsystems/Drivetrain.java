/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.controllers.PID;
import frc.robot.RobotMap;
import frc.settings.DrivetrainSettings;
import frc.util.NemesisCANEncoder;
import frc.util.NemesisMultiMC;

/**
 * Drivetrain Class for the 2019 Robot Features:
 * 
 * @author Harsh Padhye
 */
public class Drivetrain extends Subsystem implements RobotMap, DrivetrainSettings {

  // new singleton
  private static Drivetrain driveInstance = null;

  public static Drivetrain getDrivetrainInstance() {
    if (driveInstance == null) {
      driveInstance = new Drivetrain();
    }
    return driveInstance;
  }

  // enum for the switch statements, dictates the drivetrain control method
  private States driveState = States.STOPPED;

  private enum States {
    STOPPED, TELEOP_DRIVE, AUTON_DRIVE, PATH_FOLLOWING, TURN, DRIVE_STRAIGHT
  }

  // allows us to drive the robot via arcade drive
  private DifferentialDrive driveSystem;

  // motor controllers for the DT, set up in master/slave fashion
  // motor controllers closer to the front of the robot are masters
  private CANSparkMax leftDriveMaster;
  private CANSparkMax leftDriveSlave;
  private CANSparkMax rightDriveMaster;
  private CANSparkMax rightDriveSlave;

  // holds the left and right motor controllers as one speed controllers
  private NemesisMultiMC dualMotorControllers;

  // encoders for the DT, preinstalled in NEO motors
  private NemesisCANEncoder leftDriveEncoder;
  private NemesisCANEncoder rightDriveEncoder;

  // gear shifting piston (low true, high false)
  private Solenoid shiftingPiston;

  // Gyro sensor for reading the robots heading
  private ADXRS450_Gyro gyro;

  // PID controller for turning in place
  private PID turnController;

  // these variables are substitutes for teleop joystick commands
  // straight: Left Joystick Y axis; turn: Right Joystick X Axis
  private double straightPower, turnPower;

  public Drivetrain() {

    leftDriveMaster = new CANSparkMax(LEFT_DRIVE_MASTER, MotorType.kBrushless);
    leftDriveSlave = new CANSparkMax(LEFT_DRIVE_SLAVE, MotorType.kBrushless);
    rightDriveMaster = new CANSparkMax(RIGHT_DIRVE_MASTER, MotorType.kBrushless);
    rightDriveSlave = new CANSparkMax(RIGHT_DRIVE_SLAVE, MotorType.kBrushless);

    // set the slave controllers to follow their respective masters(same side)
    leftDriveSlave.follow(leftDriveMaster);
    rightDriveSlave.follow(rightDriveMaster);

    // instantiates drive encoders from their NEO motors
    leftDriveEncoder = new NemesisCANEncoder(leftDriveMaster);
    rightDriveEncoder = new NemesisCANEncoder(rightDriveMaster);

    driveSystem = new DifferentialDrive(leftDriveMaster, rightDriveMaster);
    dualMotorControllers = new NemesisMultiMC(leftDriveMaster, rightDriveMaster);

    shiftingPiston = new Solenoid(GEAR_SHIFT_SOLENOID);
    gyro = new ADXRS450_Gyro();

    turnController = new PID(TURN_KP, TURN_KI, TURN_KD, 2.0, gyro, dualMotorControllers);

    straightPower = 0.0;
    turnPower = 0.0;
  }

  // updates the drivetrain's state with every iteration of teleopPeriodic()
  public void update() {
    // switch statement controlled by driveState
    switch (driveState) {
    case STOPPED:
      setSpeeds(0, 0);
      break;

    case TELEOP_DRIVE:
      automaticGearShift();
      driveSystem.arcadeDrive(straightPower, turnPower);
      break;

    case AUTON_DRIVE:
      break;

    case PATH_FOLLOWING:
      break;

    case TURN:
      turnController.calculate();
      if (turnController.isDone()) {
        driveState = States.STOPPED;
      }
      break;

    case DRIVE_STRAIGHT:
      break;

    default:
      System.out.println("You've hit the default case in Drivetrain");
      break;

    }
  }

  /**
   * Stops the drivetrain from moving and turning
   */
  public void stopDrivetrain() {
    driveState = States.STOPPED;
  }

  /**
   * drives the robot in teleoperated mode
   * 
   * @param straight Input of Left Joystick's Y Axis
   * @param turn     Input of Right Joystick's X Axis
   */
  public void teleopDrive(double straight, double turn) {
    straightPower = straight;
    turnPower = turn;

    driveState = States.TELEOP_DRIVE;
  }

  public void turn(double setpoint) {
    turnController.setSetpoint(setpoint);
    driveState = States.TURN;
  }

  /**
   * sets the speeds for the drive motors at the same time
   * 
   * @param left  drive speed for the left motor controller
   * @param right drive speed for the right motor controller
   */
  public void setSpeeds(double leftSpeed, double rightSpeed) {
    leftDriveMaster.set(leftSpeed);
    rightDriveMaster.set(rightSpeed);
  }

  /**
   * This method reads the current speed of the drive motors and automatically
   * determines which gear the robot should be in (high = false, low = true)
   */
  public void automaticGearShift() {
    // averages the two sides to find center speed
    double robotSpeed = (leftDriveEncoder.getVelocity() + rightDriveEncoder.getVelocity()) / 2;

    shiftingPiston.set(robotSpeed < MAX_LOW_GEAR_VELOCITY); // need to see if velocity returns ft/s
  }

  /**
   * Forces drivetrain to desired gear
   * 
   * @param isLowGear true if the robot should be in low gear, false if the robot
   *                  should be in high gear
   */
  public void manualGearShift(boolean isLowGear) {
    shiftingPiston.set(isLowGear);
  }

  public DifferentialDrive getRobotDrive() {
    return driveSystem;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
