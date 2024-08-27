package frc.robot.Subsystems.Vision;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.function.Consumer;

import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Subsystems.Swerve.SwervePoint;

/**
 * class that represents a single april tag pos estimation vision system
 */
public class LocalizationVision {

    private int m_port;
    private DatagramSocket m_socket;
    private DatagramPacket m_packet;
    private Thread m_visionThread;
    private float[] m_locals = { 0, 0, 0, 0 };
    private float[] m_lastLocals = { 0, 0, 0, 0 };
    private SwervePoint m_currentPoint;
    private Consumer<SwervePoint> m_onNewPointReceived; // what to do when a packet is received


    public LocalizationVision(int port) {
        this.m_currentPoint = new SwervePoint(0, 0, 0);
        this.m_port = port;
        m_onNewPointReceived = (SwervePoint) -> {};
        try {
            //create socket
            m_socket = new DatagramSocket(m_port, InetAddress.getByName("0.0.0.0"));
            m_socket.setBroadcast(true);
            byte[] buf = new byte[48];
            m_packet = new DatagramPacket(buf, buf.length);
        } catch (Exception e) {
            e.printStackTrace();
        }

        m_visionThread = new Thread(() -> {
            while (true) {
                try {
                    m_socket.receive(m_packet);
                } catch (IOException e) {
                    e.printStackTrace();
                }
                
                // get floats from socket
                float[] new_locals = new float[] {
                        (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat()),
                        (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat(4)),
                        (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat(8)), 
                        (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat(12)), 
                };
                                                                                                              
                // save last locals
                for (int i = 0; i < m_locals.length; i++) {
                    m_lastLocals[i] = m_locals[i];
                }

                // save current locals
                for (int i = 0; i < m_locals.length; i++) {
                    m_locals[i] = new_locals[i];
                }

                m_currentPoint.setX(m_locals[0]);// camera's x
                m_currentPoint.setY(m_locals[2]);// camera's z
                m_currentPoint.setAngle(-m_locals[3]); //camera's yaw

                m_onNewPointReceived.accept(m_currentPoint);
            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }

    public void setOnNewPointReceived(Consumer<SwervePoint> onNewPointReceived){
        m_onNewPointReceived = onNewPointReceived;
    }

    public float[] get3DCords() {
        float[] newLocals = {0, 0, 0};
        for (int i = 0; i < m_locals.length; i++) {
            newLocals[i] = (m_locals[i] + m_lastLocals[i]) / 2.0f;
        }
        return m_locals;
    }

    /**
     * @return the cords from a top-down 2d perspective 
     */
    public SwervePoint get2DCords() {
        return m_currentPoint;
    }

    public double getRobotFieldAngle(){
        return m_currentPoint.getAngle();
    }

    public float getX() {
        return m_locals[0];
    }

    public float getY() {
        return m_locals[1];
    }

    public float getZ() {
        return m_locals[2];
    }

}
