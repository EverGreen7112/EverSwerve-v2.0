package frc.robot.Subsystems.Vision;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Subsystems.Swerve.SwerveLocalizator;
import frc.robot.Subsystems.Swerve.SwervePoint;

/**
 * class that represents a single april tag pos estimation vision system
 */
public class LocalizationVision {
    private final float VISION_FRAME_TIME = 0;

    private int m_port;
    private DatagramSocket m_socket;
    private DatagramPacket m_packet;
    private Thread m_visionThread;
    private float[] m_locals = { 0, 0, 0, 0 };
    private float[] m_lastLocals = { 0, 0, 0, 0 };
    private double m_robotX, m_robotY, m_robotAngle;

    public LocalizationVision(int port) {
        Swerve swerve = Swerve.getInstance();
        this.m_port = port;
        try {
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

                double angularVelocity = swerve.getAngularVelocity();
                m_robotX = m_locals[0];// camera's x
                m_robotY = m_locals[2];// camera's z
                m_robotAngle = -m_locals[3] + (angularVelocity * VISION_FRAME_TIME);

                /*
                 * we add an estimation for delta angle to the angle given by the vision
                 * / this is an attempt to compensate for the fact that the data as given by the
                 * vision is delayed
                 * this was added to help compensate for angle offset drifting
                 */
            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
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
        return new SwervePoint(m_robotX, m_robotY, m_robotAngle);
    }

    public double getRobotFieldAngle(){
        return m_robotAngle;
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
