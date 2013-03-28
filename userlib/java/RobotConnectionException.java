/**
 * Exception that will be thrown if robot connection is lost
 * or cannot be established.
 * @author zjb
 */
public class RobotConnectionException extends RuntimeException {
    
    /**
     * Constructor
     * @param msg Exception message
     */
    public RobotConnectionException(String msg) {
        super(msg);
    }
}
