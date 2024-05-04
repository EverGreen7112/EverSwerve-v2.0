package frc.robot.SensorsAndControllers.Implementations.MotorControllers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.SensorsAndControllers.EverMotorController;

/**
 * This class represents any spark based motor controller such as: sparkmax, sparkflex, etc...
 */
public class EverSpark extends EverMotorController{
    
    private CANSparkBase m_controller;

    public EverSpark(int id) {
        super(id);
        m_controller = new CANSparkMax(id, MotorType.kBrushless);
    }

    public EverSpark(int id, double kp, double ki, double kd, double kf) {
        this(id);
        setPidf(kp, ki, kd, kf);
    }

    @Override
    public double get(ControlType type) {
        switch (type) {
            case kPrecent:
                return m_controller.get();
            case kPos:
                return m_controller.getEncoder().getPosition();
            case kVel:
                return m_controller.getEncoder().getVelocity();
        }
        return m_controller.get();
    }

    @Override
    public double get() {
        return get(ControlType.kPrecent);
    }

    @Override
    public void set(ControlType type, double value) {
        switch (type) {
            case kPrecent:
                m_controller.set(value);
                break;
            case kPos:
                m_controller.getPIDController().setReference(value, com.revrobotics.ControlType.kPosition);
                break;
            case kVel:
                m_controller.getPIDController().setReference(value, com.revrobotics.ControlType.kVelocity);
                break;
        }
    }

    @Override
    public void set(double value){
        m_controller.set(value);
    }


    @Override
    public void setInverted(boolean isInverted) {
        m_controller.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return m_controller.getInverted();
    }

    @Override
    public void stop() {
        m_controller.stopMotor();
    }

    @Override
    public void setControlTypeConversionFactor(ControlType type, double factor) {
        switch (type) {
            case kPos:
                m_controller.getEncoder().setPositionConversionFactor(factor);    
                break;
            case kVel:
                m_controller.getEncoder().setVelocityConversionFactor(factor);
                break;
            default:
                break;
        }
    }

    @Override
    public void setEncoderPos(double pos) {
        m_controller.getEncoder().setPosition(pos);
    }

    @Override
    public void follow(EverMotorController motorController) {
        if(!(motorController instanceof EverSpark))
            throw new RuntimeException("a spark max cant follow a different kind of motor controller");
        m_controller.follow(m_controller);
    }

    @Override
    public int getId() {
        return m_id;
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        switch (idleMode) {
            case kBrake:
                m_controller.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
                break;
            case kCoast:
                m_controller.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
                break;
            default:
                break;
        }
    }

    @Override
    public double getTemperature() {
        return m_controller.getMotorTemperature();
    }

    @Override
    public void restoreFactoryDefaults() {
        m_controller.restoreFactoryDefaults();
    }
    
}
