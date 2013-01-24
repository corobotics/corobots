import java.util.*;
import java.io.*;

public class RobotMap {

    private static RobotMap theMap = new RobotMap();
    private HashMap<String,MapNode> nodes;

    private RobotMap() {
	try {
	    Scanner input = new Scanner(new File("../corobot_map/map/waypoints.csv"));
	    String nodeline = input.nextLine(); // toss the headers of the csv
	    nodes = new HashMap<String,MapNode>();
	    while (input.hasNextLine()) {
		nodeline = input.nextLine();
		String[] parts = nodeline.split(",");
		// name xpix ypix xm ym type nbrs...
		MapNode newnode = new MapNode();
		newnode.name = parts[0].toUpperCase();
		double x = Double.parseDouble(parts[3]);
		double y = Double.parseDouble(parts[4]);
		newnode.pos = new Point(x,y);
		newnode.nbrs = new LinkedList<String>();
		for (int i = 6; i < parts.length; i++)
		    if (!(parts[i].equals("")))
			newnode.nbrs.add(parts[i].toUpperCase());
		nodes.put(newnode.name,newnode);
	    }
	} catch (IOException e) {
	    System.err.println("Map cannot be built!");
	    System.err.println(e);
	}
    }
    
    public static TreeMap<Double,MapNode> getNodesFromLoc(Point p) {
	TreeMap<Double,MapNode> pts = new TreeMap<Double,MapNode>();
	for (MapNode n : theMap.nodes.values()) {
	    double d = p.dist(n.pos);
	    pts.put(d,n);
	}
	return pts;
    }

    public static HashMap<String,MapNode> getNodes() {
	return theMap.nodes;
    }

    public static Set<String> getNodeNames() {
	return theMap.nodes.keySet();
    }

    public static MapNode getNode(String name) {
	return theMap.nodes.get(name);
    }
}
class MapNode {
    String name;
    Point pos; // in meters
    List<String> nbrs;
}
