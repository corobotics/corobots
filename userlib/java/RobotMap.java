import java.util.*;
import java.io.*;

/**
 * Class for representing the map (waypoint graph)
 */
public class RobotMap {

    private static RobotMap theMap = new RobotMap();
    private HashMap<String,MapNode> nodes;

    /**
     * Private constructor, using the singleton pattern
     */
    private RobotMap() {
        try {
            Scanner input = new Scanner(new File("../../corobot_map/map/waypoints.csv"));
            String nodeline = input.nextLine(); // toss the headers of the csv
            nodes = new HashMap<String,MapNode>();
            while (input.hasNextLine()) {
                nodeline = input.nextLine();
                String[] parts = nodeline.split(",");
                // name xpix ypix xm ym type nbrs...
                MapNode newnode = new MapNode();
                newnode.name = parts[0].toUpperCase();
                newnode.x = Double.parseDouble(parts[3]);
                newnode.y = Double.parseDouble(parts[4]);
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

    /**
     * Returns the closest map node (waypoint) to the given (x,y) location
     * @param x X coordinate      
     * @param y Y coordinate 
     * @return Closest map node - note: may not be reachable in a straight line!
     */
    public static String getClosestNode(double x, double y) {
        double minsqdist = 99999;
        String closest = "";
        for (MapNode mn : theMap.nodes.values()) {
            double sqdist = (x - mn.x) * (x - mn.x) + (y - mn.y) * (y - mn.y);
            if (sqdist < minsqdist) {
                minsqdist = sqdist;
                closest = mn.name;
            }
        }
        return closest;
    }
    
    /**
     * Returns all map nodes (the whole graph, essentially)
     * May disappear?
     * @return Map of node name to node
     */
    public static HashMap<String,MapNode> getNodes() {
        return theMap.nodes;
    }

    /**
     * Returns a set of all node names (thus, no location or neighbor info)
     * @return set of names
     */
    public static Set<String> getNodeNames() {
        return theMap.nodes.keySet();
    }

    /**
     * Map lookup for a given name
     * @param name Node name
     * @return node with that name (or null, if no such node)
     */
    public static MapNode getNode(String name) {
        name = name.toUpperCase();
        return theMap.nodes.get(name);
    }

    /** 
     * Map lookup, testing the presence of a name
     * @param name Node name to look up
     * @return Whether that name is present in the map
     */
    public static boolean isNode(String name) {
        name = name.toUpperCase();
        return theMap.nodes.containsKey(name);
    }
}

/**
 * Simple to class to represent a node in the map.
 * contains a name, location and neighbors (all public)
 */
class MapNode {
    String name;
    double x, y; // in meters
    List<String> nbrs;
}
