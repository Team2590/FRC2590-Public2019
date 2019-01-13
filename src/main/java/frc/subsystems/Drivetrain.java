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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.settings.DrivetrainSettings;

/**
 * Drivetrain Class for the 2019 Robot
 * Features: 
 * @author Harsh Padhye
 */
public class Drivetrain extends Subsystem implements RobotMap, DrivetrainSettings{

  //new singleton
	private static Drivetrain driveInstance = null;
	public static Drivetrain getDrivetrainInstance() {
		if(driveInstance == null) {
			driveInstance = new Drivetrain();
		}
		return driveInstance;
  }
  //allows us to drive the robot via arcade drive
  DifferentialDrive driveSystem;
  
  //motor controllers for the DT, set up in master/slave fashion
  
  CANSparkMax leftDriveMaster;
  CANSparkMax leftDriveSlave;
  CANSparkMax rightDriveMaster;
  CANSparkMax rightDriveSlave;
  
  Drivetrain() {
    //Motor controllers, currently testing SPARK MCs
    
    leftDriveMaster = new CANSparkMax(left_drive_master, MotorType.kBrushless);
    leftDriveSlave = new CANSparkMax(left_drive_slave, MotorType.kBrushless);
    rightDriveMaster = new CANSparkMax(right_drive_master, MotorType.kBrushless);
    rightDriveSlave = new CANSparkMax(right_drive_slave, MotorType.kBrushless);

    //set the slave controllers to follow their respective masters(same side)
    leftDriveSlave.follow(leftDriveMaster);
    rightDriveSlave.follow(rightDriveMaster);

    //differential drive system (arcade drive is deprecated)
    driveSystem = new DifferentialDrive(leftDriveMaster, rightDriveMaster);
  }

  /**
   * sets the speeds for the drive motors at the same time
   * @param left: drive speed for the left motor controller
   * @param right: drive speed for the right motor controller
   */
  public void setSpeeds(double leftSpeed, double rightSpeed){
    leftDriveMaster.set(leftSpeed);
    rightDriveMaster.set(rightSpeed);
  }

  public DifferentialDrive getRobotDrive(){
    return driveSystem;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
