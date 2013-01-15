public class SimpleExample {

    public static void main(String[] args) {
	
	Robot robot = new Robot();
	System.out.println("Connected!");
	Point p = robot.getPos();
	System.out.println("Starting at " + p);
	if (robot.goToXY(p.getX() - 1.5, p.getY() + 1.0, true)) 
	    System.out.println("Made it!");
	else
	    System.out.println("didn't make it.");
    }
}
