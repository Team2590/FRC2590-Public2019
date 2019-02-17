package frc.looper;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Notifier;

/**
 * The handler for update loops
 * 
 * @author Connor_Hofenbitzer, Harsh Padhye
 */
public class Looper {

  private double delayT = 0;
  private Notifier notifier;
  private boolean running = false;
  private ArrayList<Loop> loopArray;

  /**
   * The runnable looper
   */
  private Runnable masterLoop = () -> {
    if (running) {
      for (Loop l : loopArray) {
        l.runLoop();
      }
    }
  };

  /**
   * Master loop class, handles all other loops
   * 
   * @param delayTime : time to wait in between cycles
   */
  public Looper(double delayTime) {
    delayT = delayTime;
    loopArray = new ArrayList<Loop>();
    notifier = new Notifier(masterLoop);
  }

  /**
   * add a new loop to the arraylist
   * 
   * @param loop : the loop method called with every iteration
   */
  public void register(Loop loop) {
    loopArray.add(loop);
  }

  /**
   * Starts all the loops
   */
  public void startLoops() {
    if (!running) {
      System.out.println("starting loops");
      running = true;
      notifier.startPeriodic(delayT);
    }
  }

  /**
   * Ends the loops
   */
  public void onEnd() {
    running = false;
  }
}
