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

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.settings.DrivetrainSettings;

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
    STOPPED, TELEOP_DRIVE, AUTON_DRIVE, PATH_FOLLOWING, TURN, DRIVE_STRAIGHT, VELOCITY
  }

  // allows us to drive the robot via arcade drive
  DifferentialDrive driveSystem;

  // motor controllers for the DT, set up in master/slave fashion
  private CANSparkMax leftDriveMaster;
  private CANSparkMax leftDriveSlave;
  private CANSparkMax rightDriveMaster;
  private CANSparkMax rightDriveSlave;

  // encoders for the DT, preinstalled in NEO motors
  private CANEncoder leftDriveEncoder;
  private CANEncoder rightDriveEncoder;

  // gear shifting piston
  private Solenoid shiftingPiston;

  // these variables are substitutes for teleop joystick commands
  // straight: Left Joystick Y axis; turn: Right Joystick X Axis
  double straightPower, turnPower;

  public Drivetrain() {
    // Motor controllers, currently testing SPARK MCs
    leftDriveMaster = new CANSparkMax(LEFT_DRIVE_MASTER, MotorType.kBrushless);
    leftDriveSlave = new CANSparkMax(LEFT_DRIVE_SLAVE, MotorType.kBrushless);
    rightDriveMaster = new CANSparkMax(RIGHT_DIRVE_MASTER, MotorType.kBrushless);
    rightDriveSlave = new CANSparkMax(RIGHT_DRIVE_SLAVE, MotorType.kBrushless);

    // set the slave controllers to follow their respective masters(same side)
    leftDriveSlave.follow(leftDriveMaster);
    rightDriveSlave.follow(rightDriveMaster);

    // instantiates drive encoders from their NEO motors
    leftDriveEncoder = leftDriveMaster.getEncoder();
    rightDriveEncoder = rightDriveMaster.getEncoder();

    driveSystem = new DifferentialDrive(leftDriveMaster, rightDriveMaster);

    // for shifting between low and high gear (low true, high false)
    shiftingPiston = new Solenoid(GEAR_SHIFT_SOLENOID);

    // setting numerical variables to the default values
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
      driveSystem.arcadeDrive(straightPower, turnPower);
      break;

    case AUTON_DRIVE:
      break;

    case PATH_FOLLOWING:
      break;

    case TURN:
      break;

    case DRIVE_STRAIGHT:
      break;

    case VELOCITY:
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
   * @param straight: Input of Left Joystick's Y Axis
   * @param turn: Input of Right Joystick's X Axis
   */
  public void teleopDrive(double straight, double turn) {
    straightPower = straight;
    turnPower = turn;

    driveState = States.TELEOP_DRIVE;
  }

  /**
   * sets the speeds for the drive motors at the same time
   * 
   * @param left: drive speed for the left motor controller
   * @param right: drive speed for the right motor controller
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
   * @param isLowGear: true if the robot should be in low gear, false if the robot
   *        should be in high gear
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
