package frc.robot.SensorsAndControllers;

import edu.wpi.first.wpilibj.SerialPort.Port;

public abstract class EverGyro{
    
    protected Port m_port;

    public EverGyro(Port port){
        m_port = port;
    }
    
    /**
     * @return yaw angle
     */
    public abstract double getYaw();
    
    /**
     * @return pitch angle
     */
    public abstract double getPitch();
    
    /**
     * @return roll angle
     */
    public abstract double getRoll();

    /**
     * reset yaw angle
     */
    public abstract void zeroYaw();
    
    /**
     * reset pitch angle
     */
    public abstract void zeroPitch();
    
    /**
     * reset roll angle
     */
    public abstract void zeroRoll();
    

    /**
     * @return acceleration in the x axis
     */
    public abstract double getAccelX();    
   
    /**
     * @return acceleration in the y axis
     */
    public abstract double getAccelY();
   
    /**
     * @return acceleration in the z axis
     */
    public abstract double getAccelZ();

}
