/**
 * Robot library in Java
 * 
 * Sketched version for simple simulator testing
 * Could be used as starting point for "real" API
 *
 * @author Z. Butler, Jan 2013
 */

/*
  Current questions:
  * How to be informed which robot to connect to?
  * What to do if robot connection can't be established?
  * Or if connection is lost?
  * Robot should detect user program finishing by socket
  being dropped - should make sure that status is updated
  to idle from running
*/

import java.util.*;
import java.io.*;
import java.net.Socket;
import java.net.UnknownHostException;
import java.awt.Image;

public class Robot {

    // this should come from config file or something?
    private static int USER_PORT = 15001;

    private Socket sock;
    private PrintWriter out;
    private BufferedReader in;

    /**
     * Constructor, starts connection to a robot...
     */
    public Robot() {
        // offloaded to another function for now mostly so that
        // Javadocs can be hidden 
        System.err.println("Connecting to robot...");
        openSocket();
        // can't think of anything else to do here?
    }

    /**
     * crap!  how does the server tell us which robot 
     * without making the user code do something?  
     * Environment var maybe?
     */
    private void openSocket() {
        try {
            String robotName = System.getenv("ROBOT");
            sock = new Socket(robotName, USER_PORT);
            out = new PrintWriter(sock.getOutputStream());
            in = new BufferedReader(new InputStreamReader(sock.getInputStream()));
        } catch (IOException e) {
            System.err.println("Error connecting to assigned robot.  Please try again.");
            throw new RobotConnectionException("in constructor");
        }
    }

    /**
     * Plans and executes a path to the given location.  Planning is done by the robot.
     *
     * @param location Name (as on map) 
     * @param block specifies whether this call blocks until location reached or some failure condition.
     * @return return whether location has been reached (if blocking)
     */
    public boolean navigateToLocation(String location, boolean block) {
        location = location.toUpperCase();
        if (RobotMap.isNode(location)) {
            out.println("NAVTOLOC " + location.toUpperCase());
            out.flush();
            if (block)
                return queryArrive();
            return true;
        }
        else {
            return false;
        }
    }
    
    /**
     * Attempts to move in a straight line to the given location.
     *
     * Currently not implemented, waiting for map.
     * @param location Name (as on map) 
     * @param block specifies whether this call blocks until location reached or some failure condition.
     * @return return whether location has been reached (if blocking)
     */
    public boolean goToLocation(String location, boolean block) {
        location = location.toUpperCase();
        if (RobotMap.isNode(location)) {
            out.println("GOTOLOC " + location.toUpperCase());
            out.flush();
            if (block)
                return queryArrive();
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Attempts to move in a straight line to the given X,Y location
     * @param x X coordinate of destination (in map coordinate system) 
     * @param y Y coordinate of destination (in map coordinate system) 
     * @param block specifies whether this call blocks until location reached or some failure condition.
     * @return return whether location has been reached (if blocking)
     */
    public boolean goToXY(double x, double y, boolean block) {
        out.println("GOTOXY " + x + " " + y);
        out.flush();
        if (block) 
            return queryArrive();
        else
            // if non-blocking, be optimistic.
            return true;
    }

    /**
     * Used by goto functions to wait for ack from robot
     */
    private boolean queryArrive() {
        out.println("QUERY_ARRIVE");
        out.flush();
        String line = "";
        try {
            do {
                line = in.readLine();
            } while (!line.equals("ARRIVE") && !line.equals("GOTOFAIL"));
        } catch (IOException e) {
            System.err.println("Lost connection with robot!");
            throw new RobotConnectionException("while waiting for robot to reach destination");
        }
        return line.equals("ARRIVE");
    }

    /**
     * Queries the robot for its current position in map coordinates
     * @return position
     */
    public Point getPos() {
        out.println("GETPOS");
        out.flush();
        String strpos = "";
        try {
            strpos = in.readLine();
        } catch (IOException e) {
            System.err.println("Lost connection with robot!");
            throw new RobotConnectionException("in getPos()");
        }
        Scanner inscan = new Scanner(strpos);
        if (!(inscan.next().equals("POS")))
            // trouble, crossed signals, what to do?
            return null;
        return new Point(inscan.nextDouble(), inscan.nextDouble());
    }

    /** 
     * Gives the named location closest to the robot's current position.
     *
     * @return Name of location
     */
    public String getClosestLoc() {
        Point p = getPos();
        return RobotMap.getClosestNode(p.getX(),p.getY());
    }

    /**
     * Returns all named locations close to the robot's current position.
     * not sure how to define "close" here, but likely to be useful
     *
     * Currently not implemented.
     *
     * @return list of nearby location names
     */
    public List<String> getAllCloseLocs() {
        throw new UnsupportedOperationException();
    }

    /**
     * Sends a message for display on the local (robot) GUI
     * @param msg Message to display (&lt; 256 chars suggested)
     */
    // show a message on the laptop GUI
    public void displayMessage(String msg) {
        if (msg.length() > 255)
            msg = msg.substring(0,255);
        out.println("DISPLAY " + msg);
        out.flush();
    }
    
    /**
     * pops up an OK button on the laptop GUI.
     * Note that any timeout longer than 2 minutes will be set to 2 minutes
     * Currently unsupported by simulated or real robots
     * @param timeout Amount of time to wait for a response (in seconds) 
     * @return whether confirmed (true) or timed out (false)
     */
    public boolean waitForConfirm(int timeout) {
        if (timeout > 120)
            timeout = 120;
        out.println("CONFIRM " + timeout);
        out.flush();
        return true;
    }
        
    // RobotMap class contains a dictionary of String->MapNode
    // MapNode contains String name, double x,y, List<String> neighbors(?)

    /**
     * Obtain a picture from one of the robot's cameras
     * Extremely not implemented at present.
     * @param whichCamera Which camera to use: 0 = left, 1 = fwd, 2 = right
     * @return some image in some format?
     */
    public Image getImage(int whichCamera) {    
        throw new UnsupportedOperationException();
    }

    // may want other access to robot data but not sure what yet.
}
