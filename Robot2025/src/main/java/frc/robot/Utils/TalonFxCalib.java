package frc.robot.Utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.EverKit.Periodic;

public class TalonFxCalib{
    
    private TalonFX m_controller;
    private int m_id;
    private Slot0Configs m_startConfig;

    public TalonFxCalib(int id, Slot0Configs startConfigs){
        m_controller = new TalonFX(id);
        m_id = id;
        m_startConfig = startConfigs;
        m_controller.getConfigurator().apply(startConfigs);
        SmartDashboard.putNumber("kp", startConfigs.kP);
        SmartDashboard.putNumber("ki", startConfigs.kI);
        SmartDashboard.putNumber("kd", startConfigs.kD);
        SmartDashboard.putNumber("ks", startConfigs.kS);
        SmartDashboard.putNumber("kv", startConfigs.kV);
        SmartDashboard.putNumber("kg", startConfigs.kG);
        SmartDashboard.putNumber("ka", startConfigs.kA);
        
    } 

    public void printInfo(){
        SmartDashboard.putNumber("talon "+ m_id +" pos", m_controller.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("talon "+ m_id +" vel", m_controller.getVelocity().getValueAsDouble());
    }

    public void activateVelPid(double setpoint){
        updatePIDValsFromDashboard();
        m_controller.setControl(new VelocityVoltage(setpoint));
    }

    public void activatePosPid(double setpoint){
        updatePIDValsFromDashboard();
        m_controller.setControl(new PositionVoltage(setpoint));
    }

    private void updatePIDValsFromDashboard(){
        Slot0Configs a = new Slot0Configs();
        a.kP = SmartDashboard.getNumber("kp", m_startConfig.kP);
        a.kI = SmartDashboard.getNumber("ki", m_startConfig.kI);
        a.kP = SmartDashboard.getNumber("kd", m_startConfig.kD);
        a.kP = SmartDashboard.getNumber("ks", m_startConfig.kS);
        a.kP = SmartDashboard.getNumber("kv", m_startConfig.kV);
        a.kP = SmartDashboard.getNumber("kg", m_startConfig.kG);
        a.kP = SmartDashboard.getNumber("ka", m_startConfig.kA);
        m_controller.getConfigurator().apply(a);
    }









}
