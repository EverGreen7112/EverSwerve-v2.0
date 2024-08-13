package frc.robot.Subsystems.Vision;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.SwerveConsts;
import frc.robot.Subsystems.SwerveLocalizator;

public class LocalizationVision {
    private final double VISION_FRAME_TIME = 0;
    
    private int m_port;
    private DatagramSocket m_socket;
    private DatagramPacket m_packet;
    private Thread m_visionThread;
    private float[] m_locals={0, 0, 0, 0};
    private float[] m_lastLocals={0, 0, 0, 0};
    
    public LocalizationVision(int port){
        Swerve swerve = Swerve.getInstance();
        this.m_port = port;
        try{ 
            m_socket = new DatagramSocket(m_port, InetAddress.getByName("0.0.0.0"));
            m_socket.setBroadcast(true);
            byte[] buf = new byte[48];
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
                        //get floats from socket
                        float[] new_locals = new float[]{(ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat()),
                            (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat(4)),//get second float
                            (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat(8)), //get third float
                            (ByteBuffer.wrap(m_packet.getData()).order(ByteOrder.LITTLE_ENDIAN).getFloat(12)),}; // get fourth float
                        //save last locals
                        for(int i=0; i<m_locals.length; i++){
                            m_lastLocals[i] = m_locals[i];
                        }
                        //save current locals
                        for(int i=0; i<m_locals.length; i++){
                            m_locals[i] = new_locals[i];
                        }
                    
                    // double angularVelocity = swerve.getAngularVelocity();
                    // SwerveLocalizator.getInstance().offsetLocalizationTo(
                    //     m_locals[0],                                                         // x
                    //     m_locals[2],                                                         // y
                    //     -m_locals[3] + (angularVelocity * VISION_FRAME_TIME));  // angle
                    //     // we add an estimation for delta angle to the angle given by the vision
                    //     // this is an attempt to compensate for the fact that the data as given by the vision is delayed
                    //     // this was added to help compensate for angle offset drifting
                    
            }
        });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }

    public float[] getXYZ(){
        // this.getLocals();
        float[] newLocals={0,0,0};
        for(int i=0; i<m_locals.length; i++){
            newLocals[i] = (m_locals[i] + m_lastLocals[i])/2;
        }
        return m_locals; 
    }

    public float getX(){
        return m_locals[0];
    }
    
    public float getY(){
        return m_locals[1];
    }

    public float getZ(){
        return m_locals[2];
    }
    
    public double getAngleY(){
        // this.getLocals();
        float[] temp=getXYZ();
        return Math.toDegrees(Math.atan(temp[0]/temp[2]))*-1;
    }

    public double getAngleX(){
        // this.getLocals();
        float[] temp=getXYZ();
        return Math.toDegrees(Math.atan(temp[1]/temp[2]))*-1;
    }    
}
