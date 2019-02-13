/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import frc.auto.NemesisRunnable;

/**
 * Follows a specified path
 * 
 * @author Connor_Hofenbitzer
 *
 */
public class DrivePath implements NemesisRunnable {

    // TrajectoryFollow follower;
    // DualTrajectory segs;
    private boolean started = false;
    private boolean reverseHeadings = false;
    private String profile;

    /**
     * Runs a path with a offset of 0 and reverse is false
     * 
     * @param profile name of the profile
     */
    public DrivePath(String profile, boolean right) {
        // init the reader and feed it the profile name
        this(profile, false, 0, right);
    }

    /**
     * Runs a path
     * 
     * @param profile path name
     * @param rev     if the heading is reversed
     * @param offset  angular offset in degrees
     * @param right   if the path is turning right
     */
    public DrivePath(String profile, boolean rev, double offset, boolean right) {

        this.profile = profile;

        // init a new reader with the given offsets and settings
        // TrajectoryReader reader = new TrajectoryReader(profile);

        // reader.setReverseHeading(rev);
        // reader.setHeadingOffset(offset);
        // segs = reader.getSegmentArray();

        // follower = new TrajectoryFollow(segs, right);
    }

    public void reversePath(boolean rev, boolean isRight) {
        // follower.reverse(rev, isRight);
    }

    @Override
    public void run() {
        // if it hasnt started the path, start driving
        if (!started) {
            // follower.start();
            // Robot.getDriveTrain().runPath();
        }

        // then tell us that we have started
        started = true;
    }

    @Override
    public boolean isDone() {
        // check if we've finished driving
        // return follower.isFinishedPath();
        return true;
    }

    /**
     * stop the path by interrupting the thread that runs it
     */
    public void cancel() {
        // follower.interrupt();
    }

    @Override
    public String getKey() {
        return "DRIVE PATH: " + profile;
    }
}
