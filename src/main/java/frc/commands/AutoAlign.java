/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import frc.auto.NemesisRunnable;
import frc.commands.DriveStraight;

/**
 * Aligns the robot in respect to the vision targets during the Sandstorm period of a match
 * @author Chinmay Savanur
 */
public class AutoAlign implements NemesisRunnable {

    /**
     * Automatically aligns the robot using the Limelight camera
     */
    public AutoAlign() {

    }

    @Override
    public void run() {
        
    }

    @Override
    public boolean isDone() {
        return true;
    }

    @Override 
    public String getKey() {
        return "AUTO ALIGN";
    }
}