package frc.robot.subsystems;

import java.io.*; 
import java.net.*; 
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;

public class GetMacAddress extends Subsystem {
	
	public static String mac;
	public static boolean bPing;	

	public GetMacAddress() {
	}

	@Override
	public void initDefaultCommand() {
	}

	// Sends ping request to a provided IP address
	public static boolean sendPingRequest(String ipAddress) throws UnknownHostException, IOException {
		InetAddress geek = InetAddress.getByName(ipAddress);
		System.out.println("Sending Ping Request to " + ipAddress);
		if (geek.isReachable(5000))
		{
			System.out.println("Host is reachable");
			bPing = true;
		}
		else
		{
			System.out.println("Sorry ! We can't reach to this host");
			
			bPing = false;
		}

		return bPing;
	}

	public static String getRIOMAC() {
		mac = "xx:xx:xx:xx:xx:xx";
		// Attempt to get the MAC address of the robot
		try {
			NetworkInterface network = NetworkInterface.getByInetAddress(InetAddress.getLocalHost());

			byte[] address = network.getHardwareAddress();

			StringBuilder sb = new StringBuilder();
			for (int i = 0; i < address.length; i++) {
				sb.append(String.format("%02X%s", address[i], (i < address.length - 1) ? ":" : ""));
			}
			mac = sb.toString();
			// System.out.println(mac);
		} catch (UnknownHostException e) {
			System.out.println("Unknown Host Exception - " + e);
		} catch (SocketException e) {
			System.out.println("Socket Exception - " + e);
		}
		/// Determines what robot we are using

		if (mac.equals("00:80:2F:17:BD:5F")) {
			// System.out.println("2020 Competition " + mac);
			mac = "2020 Competition - " + mac;
		} else {
			// System.out.println("Practice " + mac);
			mac = "2020 Practice - " + mac;
		}
		return mac;
	}

}