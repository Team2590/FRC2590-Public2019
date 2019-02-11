/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Subsystem;
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
	public static Drivetrain getDriveInstance() {
		if(driveInstance == null) {
			driveInstance = new Drivetrain();
		}
		return driveInstance;
  }

  
  private States driveState = States.STOPPED;
	private enum States {
		STOPPED , TELEOP_DRIVE , PATH_FOLLOWING , 
		TURN , DRIVE_STRAIGHT, VELOCITY
	}
  
  Victor leftDrive;
  Victor rightDrive;

  Drivetrain() {
    //Motor controllers, currently testing SPARK MCs
    leftDrive = new Victor(LEFT_DRIVE_PWM);
    rightDrive = new Victor(RIGHT_DRIVE_PWM);
  }

  /**
   * sets the speeds for the drive motors at the same time
   * @param left: drive speed for the left motor controller
   * @param right: drive speed for the right motor controller
   */
  public void setSpeeds(double left, double right){
    leftDrive.set(left);
    rightDrive.set(right);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }


  
}
