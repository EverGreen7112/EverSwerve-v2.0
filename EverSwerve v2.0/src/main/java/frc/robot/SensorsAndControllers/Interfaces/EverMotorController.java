package frc.robot.SensorsAndControllers.Interfaces;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;


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

    /**
     
     * @param type - mode type(precentage(-1 - 1), position, velocity)
     * @return current value. depending on the mode
     */
    public abstract double get(ControlType type);

    /**
     * Sets the appropriate output on the motor controller, depending on the mode.
     * @param type - control type(precentage(-1 - 1), position, velocity)
     * @param value - target value
     */
    public abstract void set(ControlType type, double value);

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
     * @param type - control type(precentage(-1 - 1), position, velocity)
     * @param factor - control type's output's factor 
     */
    public abstract void setControlTypeConversionFactor(ControlType type, double factor);
    
    /**
     * Set internal encoder position.
     */
    public abstract void setEncoderPos();

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
