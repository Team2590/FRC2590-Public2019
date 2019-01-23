package frc.commands;

import frc.auto.NemesisRunnable;

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