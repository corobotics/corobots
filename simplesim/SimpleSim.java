/**
 * SimController implements the controller in a very
 * simple simulated fashion.  For initial testing of
 * user apps only.
 * 
 * @author zjb Dec-2011, added to corobot project Jan-2013
 */

import java.io.*;
import java.net.*;
import java.util.*;
import java.awt.Color;
import java.awt.Dimension;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.net.InetAddress;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.client.HttpClient;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.client.ResponseHandler;
import org.apache.http.impl.client.BasicResponseHandler;

public class SimpleSim extends Thread {

    public static final int USER_PORT = 15001;
    public static final int MONITOR_PORT = 16001;
    public static final String SERVER_HOST = "129.21.30.80";

    private static final double DEST_EPS = 0.02;
    private static final int WIN_SCALE = 4;
    private GridMap mapWin;
    private int[][] theWorld;
    private int mapW, mapH;
    private double mpp;
    private double rX, rY;
    private LinkedList<Point> dests;
    private double destX, destY;
    private double rV = 0.1; // robot velocity, meters per second
    private int ticklen = 100; // simulation cycle time, millisec
    private long lastUpdate = 0; // time of last server update
    private String myName;
    private ResponseHandler<String> responseHandler;
    private HttpClient httpclient;

    public SimpleSim(String paramfile) throws Exception {
	myName = InetAddress.getLocalHost().getHostName() + "-sim";
	responseHandler = new BasicResponseHandler();
	httpclient = new DefaultHttpClient();

	Scanner s = new Scanner(new File(paramfile));
	if (!(s.next().equals("FILENAME")))
	    throw new Exception ("Expected FILENAME");
	//	FileInputStream mapFile = new FileInputStream(s.next());
	BufferedImage mapImage = ImageIO.read(new File(s.next()));
	if (!(s.next().equals("WIDTH")))
	    throw new Exception ("Expected WIDTH");
	mapW = s.nextInt();
	if (!(s.next().equals("HEIGHT")))
	    throw new Exception ("Expected HEIGHT");
	mapH = s.nextInt();
	if (!(s.next().equals("MPP")))
	    throw new Exception ("Expected MPP");
	mpp = s.nextDouble();
	
	theWorld = new int[mapW][mapH];
	mapWin = new GridMap(mapW*mpp,mapH*mpp,mpp*WIN_SCALE);
	for (int y = 0; y < mapH; y++) {
	    for (int x = 0; x < mapW; x++) {
		theWorld[x][y] = mapImage.getRGB(x,y) % 256;
		if (theWorld[x][y] < 0) theWorld[x][y] += 256;
		//if (x == y) System.out.println(mapImage.getRGB(x,y) + " " + theWorld[x][y]);
		mapWin.setPix(x/WIN_SCALE,y/WIN_SCALE,theWorld[x][y]);
	    }
	}
	if (!(s.next().equals("INIT_X")))
	    throw new Exception ("Expected INIT_X");
	rX = s.nextDouble();
	destX = rX;
	if (!(s.next().equals("INIT_Y")))
	    throw new Exception ("Expected INIT_Y");
	rY = s.nextDouble();
	destY = rY;
	mapWin.addColorBlob(rX,rY,Color.RED);

	Set<String> mapNames = RobotMap.getNodeNames();
	for (String n : mapNames) {
	    mapWin.setColorBlob(RobotMap.getNode(n).x, RobotMap.getNode(n).y, Color.BLUE);
	}
	
	mapWin.setSize(new Dimension((int)(mapW/WIN_SCALE),(int)(mapH/WIN_SCALE)));
	mapWin.setVisible(true);
    }

    // goes from world coordinates (meters, right-handed, 0 at center) 
    // to image coordinates (pixels, left-handed, 0 at upper left)
    private int mapAt(double x, double y) {
	int mx = (int)(x/mpp + mapW/2);
        int my = (int)(mapH/2 - y/mpp);
	if (mx < 0 || mx >= mapW || my < 0 || my >= mapH)
	    return -1;
	return theWorld[mx][my];
    }

    public void openSockets() throws Exception {
	ServerSocket userSock = new ServerSocket(USER_PORT);
	// if exception, will not get past here
	new UserListener(userSock).start();

	ServerSocket monSock = new ServerSocket(MONITOR_PORT);
	// if exception, will not get past here
	new MonitorListener(monSock).start();

    }

    private boolean atDestination() {
	double deltaX = destX - rX;
	double deltaY = destY - rY;
	return (Math.abs(deltaX) < DEST_EPS && Math.abs(deltaY) < DEST_EPS);
    }

    public void run() {
	// spin and update position based on data from user-comm thread
	while (true) {
	    if (!atDestination()) {
		double deltaX = destX - rX;
		double deltaY = destY - rY;
		double magDelta = Math.sqrt((deltaX*deltaX) + (deltaY*deltaY));
		double dist = rV * (ticklen/1000.0); // max dist to travel in one tick
		if (dist > magDelta)
		    dist = magDelta; // don't overshoot
		double newX = rX + dist*(deltaX/magDelta);
		double newY = rY + dist*(deltaY/magDelta);
		
		// test for collisions
		int obs = (mapAt(newX,newY));
		if (obs == 0)  { // CRASH!
		    // don't try to keep going
		    destX = rX;
		    destY = rY;
		} else {
		    // update both the sim and the display
		    // this next line is a hack
		    mapWin.eraseBlob(rX,rY);
		    mapWin.addColorBlob(newX,newY,Color.RED);
		    mapWin.repaint();
		    rX = newX;
		    rY = newY;
		}
	    }
	    try {
		sleep(ticklen);
	    } catch (InterruptedException e) {}
	    if (System.currentTimeMillis() - lastUpdate > 1000) {
		try {
		    lastUpdate = System.currentTimeMillis();
		    //		    System.out.println("Sending update to " + SERVER_HOST + 
		    //		       " with value " + rX + " " + rY);
		    HttpGet httpget = new HttpGet("http://"+SERVER_HOST+":8080/acceptInput?robotname=" + myName + "&" + "x=" + rX + "&y=" + rY);
		    httpclient.execute(httpget,responseHandler);
		} catch (IOException e) {
		    //System.err.println(e);
		}
	    }
	}
    }

    class UserListener extends Thread {
	private ServerSocket ss;
	public UserListener(ServerSocket ss) {
	    this.ss = ss;
	}

	public void run() {
	    while (true) {
		try {
		    System.out.println("Waiting for user connection");
		    Socket s = ss.accept();
		    // make sure it's localhost
		    PrintWriter out = new PrintWriter(s.getOutputStream());
		    BufferedReader in = new BufferedReader(new InputStreamReader(s.getInputStream()));
		    boolean tellArrive = false;
		    while(s.isConnected()) {
			// get a message
			// depending on its type, set a value or respond
			String message = in.readLine();
			if (message == null) break;
			System.out.println(message);
			String[] parts = message.split(" ");
			System.out.println("New message: " + parts[0]);
			if (parts[0].equals("GETPOS")) {
			    out.println("POS " + rX + " " + rY);
			    out.flush();
			} else if (parts[0].equals("GOTOXY")) {
			    double newX = Double.parseDouble(parts[1]);
			    double newY = Double.parseDouble(parts[2]);
			    // maybe do some sanity checking here
			    // but not right now
			    destX = newX;
			    destY = newY;
			} else if (parts[0].equals("GOTOLOC")) {
			    MapNode dnode = RobotMap.getNode(parts[1]);
			    if (dnode != null) {
				System.out.println("Going to " + parts[1] + " at " + dnode.x + "," + dnode.y);
				destX = dnode.x;
				destY = dnode.y;
			    }
			} else if (parts[0].equals("QUERY_ARRIVE")) {
			    // wait here since if we loop we will block
			    // until next message.
			    double waitX = destX, waitY = destY;
			    while (!atDestination()) {
				try {
				    Thread.sleep(1000);
				} catch (InterruptedException e) { }
			    }
			    // if crashed, destination will have changed
			    if (waitX == destX && waitY == destY)
				out.println("ARRIVE");
			    else
				out.println("GOTOFAIL");
			    out.flush();
			}
		    }
		} catch (IOException e) {
		    System.err.println("Monitor socket failure: " + e);
		}
	    }
	}
    }

    class MonitorListener extends Thread {
	private ServerSocket ss;
	public MonitorListener(ServerSocket ss) {
	    this.ss = ss;
	}

	public void run() {
	    while(true) {
		try {
		    System.out.println("Waiting for monitor connection");
		    Socket s = ss.accept();
		    // make sure it's localhost
		    PrintWriter out = new PrintWriter(s.getOutputStream());
		    while(s.isConnected()) {
			// send position to monitor
			out.println("POS " + rX + " " + rY);
			out.println("DEST " + destX + " " + destY);
			out.flush();
			// sleep
			try {
			    sleep(1000);
			} catch (InterruptedException e) { }
		    }
		} catch (IOException e) {
		    System.err.println("Monitor socket failure: " + e);
		}
	    }
	}
    }

    public static void main(String[] args) {

       	try {
	    String filename = "params.txt";
	    if (args.length > 0)
		filename = args[1];

	    SimpleSim theSim = new SimpleSim(filename);
	    
	    theSim.openSockets();

	    theSim.start(); 

	} catch (IOException e) {
	    System.out.println("IO Exception occurred: " + e);
	} catch (Exception e) {
	    System.out.println("Other exception: " + e);
	    e.printStackTrace();
	}
    }

}