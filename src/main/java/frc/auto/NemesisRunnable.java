package frc.auto;

/**
 * A runnable command template
 * ex. raising elevator, running intake, etc.
 * @author Chinmay Savanur
 */
public interface NemesisRunnable {

    /**
     * Runs given command
     */
    public void run();

    /**
     * Checks if the given command is completed
     * @return : command is finished
     */
    public boolean isDone();

    /**
     * Grabs the public key of the given command
     * @return the public key of the command (displayed on the dashboard when it is being run)
     */
    public String getKey();
}