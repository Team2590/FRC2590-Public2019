/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.controllers.MotionProfile;
import frc.controllers.PID;
import frc.robot.RobotMap;
import frc.settings.DrivetrainSettings;
import frc.util.NemesisCANEncoder;
import frc.util.NemesisDrive;
import frc.util.NemesisMultiMC;

/**
 * Drivetrain Class for the 2019 Robot
 * 
 * @author Harsh Padhye, Ritika Bhatnagar
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
  private NemesisDrive driveSystem;

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

  // Built-in PID controllers to linearize each drivetrain side
  private CANPIDController leftDriveLinearizer;
  private CANPIDController rightDriveLinearizer;

  // PID controller for turning in place
  private PID turnController;

  // Motion Profile controller for driving autonomously
  private MotionProfile leftDriveProfiler;
  private MotionProfile rightDriveProfiler;

  // these variables are substitutes for teleop joystick commands
  // straight: Left Joystick Y axis; turn: Right Joystick X Axis
  private double straightPower, turnPower;

  // accessory variable for the turn/drive straight command to see if the DT has
  // finished its control loop
  private boolean turnDone, driveStraightDone;

  public Drivetrain() {

    leftDriveMaster = new CANSparkMax(LEFT_DRIVE_MASTER, MotorType.kBrushless);
    leftDriveSlave = new CANSparkMax(LEFT_DRIVE_SLAVE, MotorType.kBrushless);
    rightDriveMaster = new CANSparkMax(RIGHT_DRIVE_MASTER, MotorType.kBrushless);
    rightDriveSlave = new CANSparkMax(RIGHT_DRIVE_SLAVE, MotorType.kBrushless);

    leftDriveMaster.setInverted(true);
    leftDriveSlave.setInverted(true);

    // set the slave controllers to follow their respective masters(same side)
    leftDriveSlave.follow(leftDriveMaster);
    rightDriveSlave.follow(rightDriveMaster);

    // instantiates drive encoders from their NEO motors
    leftDriveEncoder = new NemesisCANEncoder(leftDriveMaster);
    rightDriveEncoder = new NemesisCANEncoder(rightDriveMaster);

    // converts units of encoders
    setConversionFactors();

    // sets the drive motors to coast when neutral
    setDriveIdleModes(IdleMode.kCoast);

    driveSystem = new NemesisDrive();
    dualMotorControllers = new NemesisMultiMC(leftDriveMaster, rightDriveMaster);

    shiftingPiston = new Solenoid(GEAR_SHIFT_SOLENOID);
    // gyro = new ADXRS450_Gyro();

    // PID drive linearizers
    // leftDriveLinearizer = new CANPIDController(leftDriveMaster);
    // rightDriveLinearizer = new CANPIDController(rightDriveMaster);

    // leftDriveLinearizer.setP(LEFT_LINEAR_KP);
    // leftDriveLinearizer.setI(LEFT_LINEAR_KI);
    // rightDriveLinearizer.setP(RIGHT_LINEAR_KP);
    // rightDriveLinearizer.setI(RIGHT_LINEAR_KI);

    // turnController = new PID(TURN_KP, TURN_KI, TURN_KD, 2.0, gyro,
    // dualMotorControllers);

    // 80 in/sec^2 arbitrary accel value to avoid syntax error
    leftDriveProfiler = new MotionProfile(DRIVETRAIN_KP, DRIVETRAIN_KI, DRIVETRAIN_KV, DRIVETRAIN_KA,
        MAX_HIGH_GEAR_VELOCITY, 80, 2.0, leftDriveEncoder, leftDriveMaster);
    rightDriveProfiler = new MotionProfile(DRIVETRAIN_KP, DRIVETRAIN_KI, DRIVETRAIN_KV, DRIVETRAIN_KA,
        MAX_HIGH_GEAR_VELOCITY, 80, 2.0, rightDriveEncoder, rightDriveMaster);

    straightPower = 0.0;
    turnPower = 0.0;

    turnDone = true;
    driveStraightDone = true;
  }

  // updates the drivetrain's state with every iteration of teleopPeriodic()
  public void update() {
    switch (driveState) {
    case STOPPED:
      setSpeeds(0, 0);
      break;

    case TELEOP_DRIVE:
      automaticGearShift();
      //arcade drive
      double out[] = driveSystem.calculate(straightPower, turnPower);
      setSpeeds(out[0], out[1]);
       
      break;

    case AUTON_DRIVE:
      break;

    case PATH_FOLLOWING:
      break;

    case TURN:
      turnController.calculate();
      if (turnController.isDone()) {
        turnDone = true;
        driveState = States.STOPPED;
      } 
      break;

    case DRIVE_STRAIGHT:
      leftDriveProfiler.calculate();
      rightDriveProfiler.calculate();

      if (leftDriveProfiler.isDone() && rightDriveProfiler.isDone()) {
        driveStraightDone = true;
        driveState = States.STOPPED;
      }
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

  /**
   * Commands the drivetrain to turn in place
   * 
   * @param setpoint The angle (in degrees) to turn to
   */
  public void turn(double setpoint) {
    turnController.setSetpoint(setpoint);
    turnDone = false;
    driveState = States.TURN;
  }

  /**
   * Drives the robot in a straight line to a setpoint in high gear
   * 
   * @param setpoint How far the robot should move
   */
  public void driveStraight(double setpoint) {
    // shifts to high gear
    manualGearShift(false);

    driveStraightDone = false;
    leftDriveProfiler.setSetpoint(setpoint);
    rightDriveProfiler.setSetpoint(setpoint);
    driveState = States.DRIVE_STRAIGHT;
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

    shiftingPiston.set(robotSpeed > MAX_LOW_GEAR_VELOCITY); 
  }

  /**
   * Forces drivetrain to desired gear
   * 
   * @param isHighGear true if the robot should be in high gear, false if the
   *                   robot should be in low gear
   */
  public void manualGearShift(boolean isHighGear) {
    shiftingPiston.set(isHighGear);
  }

  /**
   * Sets the drive motors to brake or coast when stopped/idling
   * 
   * @param mode kBrake or kCoast
   */
  public void setDriveIdleModes(IdleMode mode) {
    leftDriveMaster.setIdleMode(mode);
    leftDriveSlave.setIdleMode(mode);
    rightDriveMaster.setIdleMode(mode);
    rightDriveSlave.setIdleMode(mode);
  }

  /**
   * Converts integrated encoders to the appropriate units of measurement
   */
  public void setConversionFactors() {
    // converts from rotations to distance
    leftDriveEncoder.setPositionConversionFactor((WHEEL_DIAMETER * Math.PI) / HIGH_GEAR_RATIO);
    rightDriveEncoder.setPositionConversionFactor((WHEEL_DIAMETER * Math.PI) / HIGH_GEAR_RATIO);
    // converts from RPM to velocity (in/s)
    leftDriveEncoder.setVelocityConversionFactor((WHEEL_DIAMETER * Math.PI) / (HIGH_GEAR_RATIO * 60.0));
    rightDriveEncoder.setVelocityConversionFactor((WHEEL_DIAMETER * Math.PI) / (HIGH_GEAR_RATIO * 60.0));
  }

  /**
   * Resets the DT's encoders and gyros
   */
  public void resetAllSensors() {
    // gyro.reset();
    leftDriveEncoder.setPosition(0.0);
    rightDriveEncoder.setPosition(0.0);
  }

  public NemesisDrive getRobotDrive() {
    return driveSystem;
  }

  public boolean isTurnDone() {
    return turnDone;
  }

  public boolean isDriveStraightDone() {
    return driveStraightDone;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
