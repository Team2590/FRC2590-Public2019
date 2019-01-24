package frc.commands;

import frc.auto.NemesisRunnable;

public class DriveStraight implements NemesisRunnable {

    private boolean isInitialized;
    private double howFar;
    private double angleToDrive;

    /**
     * Drives in a straight line using motion profiling
     */
    public DriveStraight(double distance) {

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
        return "STRAIGHT";
    }
}