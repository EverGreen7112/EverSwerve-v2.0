package frc.robot.Subsystems.Vision;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonHealthChecker {
    
    private int m_port;
    private DatagramSocket m_socket;
    private DatagramPacket m_packet;
    private Thread m_visionThread;
    private boolean m_healthCheck;
    
    public JetsonHealthChecker(int port){
        this.m_port = port;
        m_healthCheck = false;
        try{ 
            m_socket = new DatagramSocket(m_port, InetAddress.getByName("0.0.0.0"));
            m_socket.setBroadcast(true);
            byte[] buf = new byte[1];
            m_packet = new DatagramPacket(buf, buf.length);
        }
        catch (Exception e){
            e.printStackTrace();
        }

        m_visionThread = new Thread(()->{
            while(true){
                try {
                    m_socket.receive(m_packet);
                } catch (IOException e) {
                    e.printStackTrace();
                }       
                boolean data = (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).get() != 0);
                if(data){
                    m_healthCheck = !m_healthCheck;
                }
                SmartDashboard.putBoolean("jetson health check", m_healthCheck);
            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }

}