package frc.robot.Utils.EverKit.Implementations.MotorControllers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;


public class EverSparkMax implements EverMotorController{

    private CANSparkMax m_controller;

    public EverSparkMax(int id){
        m_controller = new CANSparkMax(id, MotorType.kBrushless);
    }

    @Override
    public double get() {
        return m_controller.get();
    }

    @Override
    public void set(double value) {
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
    public int getId() {
       return m_controller.getDeviceId();
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

    public EverSparkInternalEncoder getSparkInternalEncoder(){
        return new EverSparkInternalEncoder(this);
    }

    @Override
    public CANSparkMax getControllerInstance() {
        return m_controller;
    }
}
