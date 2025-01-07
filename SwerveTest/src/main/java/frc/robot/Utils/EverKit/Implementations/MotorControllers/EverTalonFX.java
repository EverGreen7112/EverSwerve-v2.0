package frc.robot.Utils.EverKit.Implementations.MotorControllers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.EverKit.EverMotorController;

public class EverTalonFX extends EverMotorController{ 
    
    private TalonFX m_controller;
    private double m_posConversionFactor;
    private double m_velConversionFactor;
    private TalonFXConfiguration m_config;

    public EverTalonFX(int id){
        m_controller = new TalonFX(id);
        m_posConversionFactor = 1;
        m_velConversionFactor = 1;
        m_config = new TalonFXConfiguration();
        
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
        return m_controller.getDeviceID();
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        switch (idleMode) {
            case kCoast:
                m_controller.setNeutralMode(NeutralModeValue.Coast);
                break;
            case kBrake:
                m_controller.setNeutralMode(NeutralModeValue.Brake);
                break;    
            default:
                break;
        }
    }

    @Override
    public double getTemperature() {
        return m_controller.getDeviceTemp().getValue();
    }

    @Override
    public void restoreFactoryDefaults() {
        m_config = new TalonFXConfiguration();
        m_controller.getConfigurator().apply(m_config);
    }

    @Override
    public TalonFX getControllerInstance() {
        return m_controller;
    }
    
    public void setPosConversionFactor(double factor) {
        m_posConversionFactor = factor;
    }

    public void setVelConversionFactor(double factor) {
        m_velConversionFactor = factor;
    }

    public double getVelConversionFactor(){
                SmartDashboard.putNumber("tal vel cf", m_velConversionFactor);

        return m_velConversionFactor;
    }

    public double getPosConversionFactor(){
        return m_posConversionFactor;
    }

}
