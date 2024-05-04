package frc.robot.Utils.SensorsAndControllers;

public abstract class EverAbsEncoder {
    
    protected int m_id;

    public EverAbsEncoder(int id){
        m_id = id;
    }
    
    /**
     * @return current position
     */
    public abstract double getPos();
    
    /**
     * Set position of encoder
     */
    public abstract void setPos(double pos);

    /**
     * @return current velocity
     */
    public abstract double getVel();
    
    /**
     * @return the offset of the encoder from origin
     * This value is set by user, using the {@link #setOffset(double)} function.
     */
    public abstract double getOffset();
    
    /**
     * Set offset of encoder from origin.
     */
    public abstract void setOffset(double offset);
    
    /**
     * Set conversion factor of the position value that is returned from the {@link #getPos()} function.
     * This should be used when trying to switch between units of measure.
     */
    public abstract void setPosConversionFactor(double factor);

    /**
     * Set conversion factor of the vel value that is returned from the {@link #getVel()}  function.
     * This should be used when trying to switch between units of measure.
     */
    public abstract void setVelConversionFactor(double factor);    
}
