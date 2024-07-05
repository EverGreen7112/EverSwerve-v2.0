package frc.robot.Utils.SensorsAndControllers;

public abstract class EverMotorController {    
    
    public enum IdleMode{
        kCoast,
        kBrake
    }
 
    public enum ControlType{
        kPrecent,
        kPos,
        kVel
    }

    //motor controller id
    protected int m_id;

    //pidf coefficients
    protected double m_kp;
    protected double m_ki;
    protected double m_kd;
    protected double m_kf;

    public EverMotorController(int id){
        m_id = id;
        setPidf(0, 0, 0, 0);
    }

    public EverMotorController(int id, double kp, double ki, double kd, double kf){
        this(id);
        setPidf(kp, ki, kd, kf);
    }

    /**
     * @param type - mode type(precentage(-1 - 1), </n>
     *                         position(if conversion factor hasn't been changed it's in rotations), 
     *                         velocity((if conversion factor hasn't been changed it's in rpm))
     * @return current value. depending on the mode
     */
    public abstract double get(ControlType type);

    /**
     * get current speed(precentage (-1 - 1))
     */
    public abstract double get();

    /**
     * Sets the appropriate output on the motor controller, depending on the mode.
      * @param type - mode type(precentage(-1 - 1), </n>
     *                         position(if conversion factor hasn't been changed it's in rotations), 
     *                         velocity((if conversion factor hasn't been changed it's in rpm))
     * @param value - target value
     */
    public abstract void set(ControlType type, double value);

    /**
     * set current speed(precentage (-1 - 1))
     */
    public abstract void set(double value);

    /**
     * @param kp - proportional coefficient
     * @param ki - integral coefficient
     * @param kd - derivative coefficient
     * @param kf - feedforward coefficient
     */
    public void setPidf(double kp, double ki, double kd, double kf){
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
        m_kf = kf;
    }

    /**
     * Invert directions of motor;
     */
    public abstract void setInverted(boolean isInverted);
    
    /**
     * @return if the directions of the motors are inverted.
     */
    public abstract boolean getInverted();
    
    /**
     * stop motor
     */
    public abstract void stop();

    /**
     * @param type - @param type - mode type(precentage(-1 - 1), </n>
     *                         position(if conversion factor hasn't been changed it's in rotations), 
     *                         velocity((if conversion factor hasn't been changed it's in rpm))
     * @param factor - control type's output's factor 
     */
    public abstract void setConversionFactor(ControlType type, double factor);
    
    /**
     * Set internal encoder position.
     */
    public abstract void setEncoderPos(double pos);

    /**
     * follow a given motor controller. 
     * output = target's output
     * @param motorController - target
     */
    public abstract void follow(EverMotorController motorController);

    /**
     * @return this motor controller's id.
     */
    public abstract int getId();

    /**
     * Set idle mode.
     * @param idleMode - coast or brake.
     */
    public abstract void setIdleMode(IdleMode idleMode);

    /**
     * @return current temperature of motor.
     */
    public abstract double getTemperature();

    /**
     * Restore factory default.
     */
    public abstract void restoreFactoryDefaults();
}
