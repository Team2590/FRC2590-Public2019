/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

/**
 * @author Harsh Padhye
 */
public class Delayer {

    private boolean isInDelay;
    private int delayCounter;
    private int delayCap;

    public Delayer() {
        isInDelay = false;
        delayCounter = 0;
        delayCap = 0;
    }

    /**
     * @param delayCap delay in seconds
     */
    public void startDelay(double delayCap) {
        this.delayCap = (int)(delayCap * 50);
        delayCounter = 0;
        isInDelay = true;
    }

    public void stopDelay() {
        delayCounter = 0;
        isInDelay = false;
    }

    public boolean checkDelayStatus() {
        if(getDelayState()) { //if the system is in a delay
            if(delayCounter > delayCap) {
                stopDelay();
                return true;
            } else {
                delayCounter++;
                return false;
            }
        } else {
            return false;
        }
    }

    public boolean getDelayState() {
        return isInDelay;
    }

    public int getDelayCount() {
        return delayCounter;
    }
}
