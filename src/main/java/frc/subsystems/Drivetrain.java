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

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.controllers.MotionProfile;
import frc.controllers.PID;
import frc.controllers.SteeringGuidance;
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
    STOPPED, TELEOP_DRIVE, GUIDE_STEERING, AUTON_DRIVE, PATH_FOLLOWING, TURN, DRIVE_STRAIGHT
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

  // encoders for the DT, attached directly to the turning shaft
  private Encoder leftExternalEnc;
  private Encoder rightExternalEnc;

  // gear shifting piston (low true, high false)
  private Solenoid shiftingPiston;

  // Gyro sensor for reading the robots heading
  private AnalogGyro gyro;

  // PDP for monitoring current draw
  private PowerDistributionPanel pdp;

  // Built-in PID controllers to linearize each drivetrain side
  private CANPIDController leftDriveLinearizer;
  private CANPIDController rightDriveLinearizer;

  // PID controller for turning in place
  private PID turnController;

  private SteeringGuidance steeringGuidance;

  // Motion Profile controller for driving autonomously
  private MotionProfile leftDriveProfiler;
  private MotionProfile rightDriveProfiler;

  // these variables are substitutes for teleop joystick commands
  // straight: Left Joystick Y axis; turn: Right Joystick X Axis
  private double straightPower, turnPower;

  // accessory variable for the turn/drive straight command to see if the DT has
  // finished its control loop
  private boolean turnDone, driveStraightDone;
  private boolean isHighGear;

  private double z, x, yaw, horizontalOffset;

  public Drivetrain() {

    leftDriveMaster = new CANSparkMax(LEFT_DRIVE_MASTER, MotorType.kBrushless);
    leftDriveSlave = new CANSparkMax(LEFT_DRIVE_SLAVE, MotorType.kBrushless);
    rightDriveMaster = new CANSparkMax(RIGHT_DRIVE_MASTER, MotorType.kBrushless);
    rightDriveSlave = new CANSparkMax(RIGHT_DRIVE_SLAVE, MotorType.kBrushless);

    leftDriveMaster.setInverted(true);
    leftDriveSlave.setInverted(true);

    // set the slave controllers to follow their respective masters(same side)
    // leftDriveSlave.follow(leftDriveMaster);
    // rightDriveSlave.follow(rightDriveMaster);

    // instantiates drive encoders from their NEO motors
    leftDriveEncoder = new NemesisCANEncoder(leftDriveMaster);
    rightDriveEncoder = new NemesisCANEncoder(rightDriveMaster);

    leftExternalEnc = new Encoder(LEFT_DRIVE_ENCODER_A, LEFT_DRIVE_ENCODER_B);
    rightExternalEnc = new Encoder(RIGHT_DRIVE_ENCODER_A, RIGHT_DRIVE_ENCODER_B);

    // converts units of encoders
    setConversionFactors();

    // sets the drive motors to coast when neutral
    setDriveIdleModes(IdleMode.kCoast);

    driveSystem = new NemesisDrive();
    dualMotorControllers = new NemesisMultiMC(leftDriveMaster, rightDriveMaster);
    dualMotorControllers.setIndividualInverted(0, true);

    shiftingPiston = new Solenoid(GEAR_SHIFT_SOLENOID);

    gyro = new AnalogGyro(DRIVETRAIN_GYRO);

    pdp = new PowerDistributionPanel();

    // PID drive linearizers
    // leftDriveLinearizer = new CANPIDController(leftDriveMaster);
    // rightDriveLinearizer = new CANPIDController(rightDriveMaster);

    // leftDriveLinearizer.setP(LEFT_LINEAR_KP);
    // leftDriveLinearizer.setI(LEFT_LINEAR_KI);
    // rightDriveLinearizer.setP(RIGHT_LINEAR_KP);
    // rightDriveLinearizer.setI(RIGHT_LINEAR_KI);

    PIDSource[] sources = { leftDriveEncoder, rightDriveEncoder };

    turnController = new PID(TURN_KP, TURN_KI, TURN_KD, TURN_TOLERANCE, gyro, dualMotorControllers);

    steeringGuidance = new SteeringGuidance(STEERING_KP, STEERING_KI, STEERING_KD, sources);

    // 80 in/sec^2 arbitrary accel value to avoid syntax error
    leftDriveProfiler = new MotionProfile(DRIVETRAIN_KP, DRIVETRAIN_KI, DRIVETRAIN_KV, DRIVETRAIN_KA,
        MAX_HIGH_GEAR_VELOCITY, 80, 2.0, leftDriveEncoder, leftDriveMaster);
    rightDriveProfiler = new MotionProfile(DRIVETRAIN_KP, DRIVETRAIN_KI, DRIVETRAIN_KV, DRIVETRAIN_KA,
        MAX_HIGH_GEAR_VELOCITY, 80, 2.0, rightDriveEncoder, rightDriveMaster);

    straightPower = 0.0;
    turnPower = 0.0;

    turnDone = true;
    driveStraightDone = true;
    isHighGear = false;
  }

  // updates the drivetrain's state with every iteration of teleopPeriodic()
  public void update() {
    switch (driveState) {
    case STOPPED:
      setSpeeds(0, 0);
      break;

    case TELEOP_DRIVE:
      // automaticGearShift();
      // arcade drive
      double out[] = driveSystem.calculate(straightPower, turnPower);
      setSpeeds(out[0], out[1]);

      break;

    case GUIDE_STEERING:
      double steeringTurnPower = steeringGuidance.calculate(z, x, yaw, horizontalOffset);

      System.out.println(yaw + " " + horizontalOffset + " " + (yaw - horizontalOffset) + " " + steeringTurnPower);

      // replace turnPower with calculation from the steering guidance controller
      double output[] = driveSystem.calculate(straightPower, steeringTurnPower);
      setSpeeds(output[0], output[1]);

      break;

    case AUTON_DRIVE:
      break;

    case PATH_FOLLOWING:
      break;

    case TURN:
      turnController.calculate();
      if (turnController.isDone()) {
        turnDone = true;
        driveState = States.TELEOP_DRIVE;
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

  public void guideSteering(double straight, double z, double x, double yaw, double horizontalOffset) {
    straightPower = straight;

    this.z = z;
    this.x = x;
    this.yaw = yaw;
    this.horizontalOffset = horizontalOffset;

    driveState = States.GUIDE_STEERING;
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
    manualGearShift(true);

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
    leftDriveSlave.set(leftSpeed);

    rightDriveMaster.set(rightSpeed);
    rightDriveSlave.set(rightSpeed);
  }

  /**
   * This method reads the current speed of the drive motors and automatically
   * determines which gear the robot should be in (high = false, low = true)
   * Upshifts at a higher speed than it Downshifts (Hysteresis)
   */
  public void automaticGearShift() {
    // averages the two sides to find center speed
    double robotSpeed = Math.abs((leftExternalEnc.getRate() + rightExternalEnc.getRate()) / 2);

    if (!isHighGear) { // currently low gear, upshifting
      isHighGear = robotSpeed > UPSHIFT_SPEED; // current calculations should flip the logic
    } else { // currently high gear, downshifting
      isHighGear = robotSpeed < DOWNSHIFT_SPEED;
    }

    // System.out.println(isHighGear);
    shiftingPiston.set(isHighGear);
  }

  /**
   * Forces drivetrain to desired gear
   * 
   * @param isHighGear true if the robot should be in high gear, false if the
   *                   robot should be in low gear
   */
  public void manualGearShift(boolean isHighGear) {
    this.isHighGear = isHighGear;
    shiftingPiston.set(isHighGear);
  }

  // toggles between high and low gear
  public void toggleShift() {
    System.out.println("Drive Gear :: " + !isHighGear);
    manualGearShift(!isHighGear);
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
    double circumference = WHEEL_DIAMETER * Math.PI;
    // converts from rotations to distance
    leftDriveEncoder.setPositionConversionFactor(circumference / HIGH_GEAR_RATIO);
    rightDriveEncoder.setPositionConversionFactor(circumference / HIGH_GEAR_RATIO);
    // converts from RPM to velocity (in/s)
    leftDriveEncoder.setVelocityConversionFactor(circumference / (HIGH_GEAR_RATIO * 60.0));
    rightDriveEncoder.setVelocityConversionFactor(circumference / (HIGH_GEAR_RATIO * 60.0));
    // pulses every 1 degree of rotation
    leftExternalEnc.setDistancePerPulse(circumference / 360.0);
    rightExternalEnc.setDistancePerPulse(circumference / 360.0);
  }

  public double getDistanceNEO(boolean left) {
    if (left) {
      return leftDriveEncoder.getPosition();
    } else {
      return rightDriveEncoder.getPosition();
    }
  }

  public double getDistanceQuadEnc(boolean left) {
    if (left) {
      return leftExternalEnc.getDistance();
    } else {
      return rightExternalEnc.getDistance();
    }
  }

  public double getHeading() {
    return gyro.getAngle();
  }

  /**
   * Resets the DT's encoders and gyros
   */
  public void resetAllSensors() {
    gyro.reset();
    leftDriveEncoder.setPosition(0.0);
    rightDriveEncoder.setPosition(0.0);
    leftExternalEnc.reset();
    rightExternalEnc.reset();
  }

  public void forceTeleop() {
    turnController.setDone();
    turnDone = true;
    driveState = States.TELEOP_DRIVE;
  }

  /**
   * @return The average current draw between the four drive motors
   */
  public double getAverageCurrentDraw() {
    return (pdp.getCurrent(LM_PWM) + pdp.getCurrent(LS_PWM) + pdp.getCurrent(RM_PWM) + pdp.getCurrent(RS_PWM) / 4.0);
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
