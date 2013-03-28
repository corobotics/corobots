import java.net.Socket;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.IOException;
import javax.swing.*;
import java.awt.*;

/**
 * Simple monitor for the simulator (or whoever)
 */
public class SimMonitor extends JFrame implements Runnable {

    private BufferedReader in;
    private JTextField xField, yField, dxField, dyField;
    private JTextArea displayArea;

    /**
     * Constructor, opens a socket to the simulator
     */
    public SimMonitor() {
	setTitle("Simple Sim Monitor");
	JPanel pospanel = new JPanel();
	pospanel.add(new JLabel("X: "));
	xField = new JTextField(15);
	xField.setEditable(false);
	pospanel.add(xField);
	pospanel.add(new JLabel("Y: "));
	yField = new JTextField(15);
	yField.setEditable(false);
	pospanel.add(yField);
	add(pospanel, BorderLayout.NORTH);

	JPanel destpanel = new JPanel();
	destpanel.add(new JLabel("DEST X: "));
	dxField = new JTextField(15);
	dxField.setEditable(false);
	destpanel.add(dxField);
	destpanel.add(new JLabel("Y: "));
	dyField = new JTextField(15);
	dyField.setEditable(false);
	destpanel.add(dyField);
	add(destpanel, BorderLayout.SOUTH);

	displayArea = new JTextArea(10,40);
	displayArea.setEditable(false);
	add(displayArea);
	
	try {
	    Socket s = new Socket("localhost",SimpleSim.MONITOR_PORT);
	    in = new BufferedReader(new InputStreamReader(s.getInputStream()));
	} catch (IOException e) {
	    displayArea.setText("Unable to establish connection to robot");
	}
    }

    /**
     * Main loop, listens to the socket and updates the GUI.
     */    
    public void run() {
	while(true) {
	    //System.out.println("Waiting for messages on " + in);
	    try {
		String l = in.readLine();
		System.out.println(l);
		String[] parts = l.split(" ");
		if (parts[0].equals("POS")) {
		    xField.setText(parts[1]);
		    yField.setText(parts[2]);
		} else if (parts[0].equals("DEST")) {
		    dxField.setText(parts[1]);
		    dyField.setText(parts[2]);
		} else if (parts[0].equals("DISPLAY")) {
		    displayArea.append("\n");
		    for (int i = 1; i < parts.length; i++) {
			displayArea.append(parts[i] + " ");
		    }
		}
	    } catch (IOException e) {
		displayArea.setText("Lost connection to robot");
	    }
	}
    }

    public static void main(String[] args) {
	SimMonitor sm = new SimMonitor();
	sm.pack();
	new Thread(sm).start();
	sm.setVisible(true);
    }
}