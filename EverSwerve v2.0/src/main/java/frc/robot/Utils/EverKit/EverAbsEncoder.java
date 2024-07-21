 package frc.robot.Utils.EverKit;

public interface EverAbsEncoder extends EverEncoder{
    
    /**
     * @return current absolute position
     */
    public double getAbsPos();
    
    /**
     * @return the offset of the encoder from origin
     * This value is set by user, using the {@link #setOffset(double)} function.
     */
    public double getOffset();
    
    /**
     * Set offset of encoder from origin.
     */
    public void setOffset(double offset);
    
}
