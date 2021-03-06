/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import frc.auto.NemesisRunnable;
import frc.robot.Robot;

public class DriveStraight implements NemesisRunnable {

    private boolean hasStarted;
    private double setpoint;

    /**
     * Drives in a straight line using motion profiling
     * @param distance distance to travel to in inches
     */
    public DriveStraight(double distance) {
        setpoint = distance;
        hasStarted = false;
    }

    @Override
    public void run() {
        // if the robot has not started driving, begin driving straight
        if(!hasStarted) { 
            Robot.getDrivetrainInstance().driveStraight(setpoint);
            hasStarted = true;
        }
    }

    @Override
    public boolean isDone() {
        return hasStarted && Robot.getDrivetrainInstance().isDriveStraightDone();
    }

    @Override 
    public String getKey() {
        return "STRAIGHT";
    }
}