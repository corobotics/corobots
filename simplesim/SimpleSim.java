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
    private Point rPos;
    private LinkedList<Point> dests; // remaining destinations (not including dest)
    private Point dest;
    private double rV = 0.1; // robot velocity, meters per second
    private int ticklen = 100; // simulation cycle time, millisec

    private long lastUpdate = 0; // time of last server update
    private String myName;
    private ResponseHandler<String> responseHandler;
    private HttpClient httpclient;

    private String monitorText = "";
    private boolean monitorConfirm = false, monitorConfirmed = false;
    private long monitorTime = 0;

    /**
     * Constructor, builds the simulated world from a file
     *
     * @param paramfile File name with simulation info
     */
    public SimpleSim(String paramfile) throws Exception {
        // try our best to create a unique name for this simulated robot
        myName = InetAddress.getLocalHost().getHostName() + "-sim-" + (System.currentTimeMillis()%10000);
        // to send info back to the server
        responseHandler = new BasicResponseHandler();
        httpclient = new DefaultHttpClient();
        
        dests = new LinkedList<Point>();

        Scanner s = new Scanner(new File(paramfile));
        if (!(s.next().equals("FILENAME")))
            throw new Exception ("Expected FILENAME");
        //  FileInputStream mapFile = new FileInputStream(s.next());
        // Image reading is not guaranteed to be general; it should
        //   work for Golisano 3rd floor image
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
        double rX = s.nextDouble();
        if (!(s.next().equals("INIT_Y")))
            throw new Exception ("Expected INIT_Y");
        double rY = s.nextDouble();
        rPos = new Point(rX,rY);
        dest = new Point(rX,rY); // separate point object, robot will move!

        mapWin.addColorBlob(rX,rY,Color.RED);

        // We will draw the waypoints here for navigation clarity
        Set<String> mapNames = RobotMap.getNodeNames();
        for (String n : mapNames) {
            mapWin.setColorBlob(RobotMap.getNode(n).pos.getX(), RobotMap.getNode(n).pos.getY(), Color.BLUE);
        }
        
        mapWin.setSize(new Dimension((int)(mapW/WIN_SCALE),(int)(mapH/WIN_SCALE)));
        mapWin.setVisible(true);
    }

    /**
     * Transforms a point from world coordinates 
     *   (meters, right-handed, 0 at center) 
     * to image coordinates (pixels, left-handed, 0 at upper left)
     *
     * @param x X location
     * @param y Ylocation
     */
    private int mapAt(double x, double y) {
        int mx = (int)(x/mpp + mapW/2);
            int my = (int)(mapH/2 - y/mpp);
        if (mx < 0 || mx >= mapW || my < 0 || my >= mapH)
            return -1;
        return theWorld[mx][my];
    }

    /**
     * Opens sockets to the server (user) and monitor
     */
    public void openSockets() throws Exception {
        ServerSocket userSock = new ServerSocket(USER_PORT);
        // if exception, will not get past here
        new UserListener(userSock).start();

        ServerSocket monSock = new ServerSocket(MONITOR_PORT);
        // if exception, will not get past here
        new MonitorListener(monSock).start();

    }

    /**
     * Draws a straight line from robot's current position and
     *  sees whether the straight line is obstacle-free
     *
     * @param n Node to draw the line to
     * @return Navigable or not
     */
    private boolean navigableTo(MapNode n) {
        double cx = rPos.getX(), cy = rPos.getY();
        double dx = n.pos.getX()-rPos.getX(), dy = n.pos.getY()-rPos.getY();
        double sdx = Math.signum(dx), sdy = Math.signum(dy);
        double incx, incy;
        if (Math.abs(dx) > Math.abs(dy)) {
            incx = sdx*mpp/2; incy = dy/(dx/incx);
        } else {
            incy = sdy*mpp/2; incx = dx/(dy/incy);
        }
        //System.out.println("Testing going straight to " + n.name + " " + n.pos + " inc (" + 
        //         incx + "," + incy + ") totald (" + dx + "," + dy + ")");
        while (sdx*dx > 0 || sdy*dy > 0) {
            if (mapAt(cx+dx,cy+dy) == 0) {
                return false;
            }
            dx -= incx; dy -= incy;
        }
        return true;
    }

    /**
     * Contains data needed to do the A* search
     */
    class AStarNode implements Comparable<AStarNode>{
        double f,g,h;
        MapNode n;
        AStarNode(double g, double h, MapNode n) {
            this.g = g; this.h = h; this.n = n; this.f = g+h;
        }
        public int compareTo(AStarNode o) {
            return (new Double(f)).compareTo(o.f);
        }
        public String toString() {
            return "" + f + " = " + (int)(g) + " + " + (int)(h) + " (" + n.name + ")";
        }
    }

    /**
     * Standard A* search.  Search starts at any node visible from
     *  robot's current location.
     * 
     * @param dest Destination of search
     * @return List of nodes along the path
     */
    private LinkedList<MapNode> aStar(MapNode dest) {
        // first find the start of the search
        System.out.println("Planning to " + dest.name);
        TreeMap<Double,MapNode> nodes = RobotMap.getNodesFromLoc(rPos);
        System.out.println("Finding possible start nodes");
        HashMap<String,String> preds = new HashMap<String,String>();
        PriorityQueue<AStarNode> queue = new PriorityQueue<AStarNode>();
        // This should probably leave out office nodes?
        for (MapNode n : nodes.values()) {
            if (navigableTo(n)) {
                //System.out.println("Adding " + n.name + " as possible 1st waypoint");
                // g is dist to waypoint, h is distance from waypoint to goal
                queue.add(new AStarNode(rPos.dist(n.pos),n.pos.dist(dest.pos),n));
                preds.put(n.name,null);
            }
        }
        // should always have a node visible?
        if (queue.isEmpty())
            throw new RuntimeException("Crap, can't find a nearby waypoint to start A*");
        while (!queue.isEmpty()) {
            AStarNode curr = queue.poll();
            MapNode cnode = curr.n;
            if (cnode.name.equals(dest.name)) {
                // got it, build path
                LinkedList<MapNode> path = new LinkedList<MapNode>();
                MapNode pnode = dest;
                while (pnode != null) {
                    path.offerFirst(pnode);
                    pnode = RobotMap.getNode(preds.get(pnode.name));
                }
                // and return
                return path;
            } 
            for (String n : cnode.nbrs) {
                MapNode adj = RobotMap.getNode(n);
                double newg = curr.g + adj.pos.dist(cnode.pos);
                double newh = adj.pos.dist(dest.pos);
                // is this an improvement?
                boolean skipit = false, replace = false;
                for (Iterator<AStarNode> qit = queue.iterator(); qit.hasNext(); ) {
                    AStarNode asn = qit.next();
                    if (asn.n == adj) {
                        if (newg+newh < asn.f) {
                            // it's better, replace (annoying to replace below since 
                            qit.remove();               
                            replace = true;
                        } else {
                            skipit = true;
                        }
                        break;
                    }
                }
                if (skipit) continue;
                // if already seen and not replacing, skip it also
                if (!replace && preds.containsKey(adj.name)) continue;
                AStarNode newnode = new AStarNode(newg,newh,adj);
                queue.offer(newnode);
                preds.put(adj.name,cnode.name);
            }
        }
        // no path!
        return null;
    }

    /**
     * Just a test to see if the robot is at its current destination
     */
    private boolean atDestination() {
        double deltaX = dest.getX() - rPos.getX();
        double deltaY = dest.getY() - rPos.getY();
        return (Math.abs(deltaX) < DEST_EPS && Math.abs(deltaY) < DEST_EPS);
    }
    
    /** 
     * Main loop of simulation
     *
     */
    public void run() {
        // spin and update position based on data from user-comm thread
        while (true) {
            if (!atDestination()) {
            double deltaX = dest.getX() - rPos.getX();
            double deltaY = dest.getY() - rPos.getY();
            double magDelta = Math.sqrt((deltaX*deltaX) + (deltaY*deltaY));
            double dist = rV * (ticklen/1000.0); // max dist to travel in one tick
            if (dist > magDelta)
                dist = magDelta; // don't overshoot
            double newX = rPos.getX() + dist*(deltaX/magDelta);
            double newY = rPos.getY() + dist*(deltaY/magDelta);
            
            // test for collisions
            int obs = (mapAt(newX,newY));
            if (obs == 0)  { // CRASH!
                // don't try to keep going
                dest = new Point(rPos);
                dests.clear();
            } else {
                // update both the sim and the display
                // this next line is a hack
                mapWin.eraseBlob(rPos.getX(),rPos.getY());
                mapWin.addColorBlob(newX,newY,Color.RED);
                mapWin.repaint();
                rPos = new Point(newX,newY);
            }
            } else if (!dests.isEmpty()) {
            // at a destination, but more to get to...
            dest = dests.poll();
            }
            try {
            sleep(ticklen);
            } catch (InterruptedException e) {}
            if (System.currentTimeMillis() - lastUpdate > 1000) {
                try {
                    lastUpdate = System.currentTimeMillis();
                    HttpGet httpget = new HttpGet("http://"+SERVER_HOST+":8080/acceptInput?robotname=" + 
                                  myName + "&" + "x=" + rPos.getX() + "&y=" + rPos.getY());
                    httpclient.execute(httpget,responseHandler);
                } catch (IOException e) {
                    //System.err.println(e);
                }
            }
        }
    }

    /**
     * Class for listening to the user socket, where the commands come in.
     */
    class UserListener extends Thread {
        private ServerSocket ss;
        public UserListener(ServerSocket ss) {
            this.ss = ss;
        }

        /**
         * Main loop
         */
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
                            out.println("POS " + rPos.getX() + " " + rPos.getY());
                            out.flush();
                        } else if (parts[0].equals("DISPLAY")) {
                            monitorText = message;
                        } else if (parts[0].equals("CONFIRM")) {
                            monitorConfirm = true;
                            int waiting = Integer.parseInt(parts[1]);
                            monitorTime = System.currentTimeMillis() + 1000 * waiting;
                        } else if (parts[0].equals("GOTOXY")) {
                            double newX = Double.parseDouble(parts[1]);
                            double newY = Double.parseDouble(parts[2]);
                            // maybe do some sanity checking here
                            // but not right now
                            dest = new Point(newX, newY);
                        } else if (parts[0].equals("GOTOLOC")) {
                            MapNode dnode = RobotMap.getNode(parts[1]);
                            // should have been checked client-side, but to be safe:
                            if (dnode != null) {
                                System.out.println("Going to " + parts[1] + " at " + dnode.pos);
                                dest = new Point(dnode.pos);
                            }
                        } else if (parts[0].equals("NAVTOLOC")) {
                            MapNode dnode = RobotMap.getNode(parts[1]);
                            if (dnode != null) {
                                List<MapNode> path = aStar(dnode);
                                if (path == null)
                                    System.err.println("Path search failed!");
                                else {
                                    System.out.println("Going to " + parts[1] + " using the path:");
                                    for (MapNode n : path) {
                                        System.out.println(n.name + " at " + n.pos);
                                        dests.add(new Point(n.pos));
                                    }
                                }
                            }
                        } else if (parts[0].equals("QUERY_ARRIVE")) {
                            // wait here since if we loop we will block
                            // until next message.
                            Point waitFor = dest;
                            // if we're navigating, wait until the end of the navigation
                            if (!dests.isEmpty()) {
                                waitFor = dests.getLast();
                            }
                            // if crashing, will be at destination and further dests deleted
                            // so even if crashing along a plan, this will become false
                            while (!atDestination() || dests.contains(waitFor)) {
                                try {
                                    Thread.sleep(1000);
                                } catch (InterruptedException e) { }
                            }
                            // if crashed, destination will have changed
                            // otherwise, the current destination will be the one we waited for
                            //   (whether the only or the last of several)
                            if (dest == waitFor)
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

    /**
     * Class for communication with a monitor
     */
    class MonitorListener extends Thread {
        private ServerSocket ss;
        public MonitorListener(ServerSocket ss) {
            this.ss = ss;
        }
        
        /**
         * Main loop for monitor communication
         */
        public void run() {
            while(true) {
                try {
                    System.out.println("Waiting for monitor connection");
                    Socket s = ss.accept();
                    // make sure it's localhost
                    System.out.println("Obtained monitor connection");
                    PrintWriter out = new PrintWriter(s.getOutputStream());
                    while(s.isConnected()) {
                        // send position to monitor
                        out.println("POS " + rPos.getX() + " " + rPos.getY());
                        out.println("DEST " + dest.getX() + " " + dest.getY());
                        out.flush();
                        if (monitorText != "") {
                            out.println(monitorText);
                            monitorText = "";
                        }
                        if (monitorConfirm) {
                            out.println("CONFIRM"); // this needs to be asynchronous...
                        }
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
    

    /**
     * Entry point
     * @param args File name for simulation parameters (optional)
     */
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
