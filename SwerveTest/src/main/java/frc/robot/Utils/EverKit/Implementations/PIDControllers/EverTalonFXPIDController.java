package frc.robot.Utils.EverKit.Implementations.PIDControllers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;

public class EverTalonFXPIDController extends EverPIDController{

    private TalonFX m_controller;
    private EverTalonFX m_everController;
    
    public EverTalonFXPIDController(EverTalonFX controller){
        m_controller = controller.getControllerInstance();
        m_everController = controller;
    }

    @Override
    public void setPIDF(double kp, double ki, double kd, double kf) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kp; 
        slot0Configs.kI = ki;
        slot0Configs.kD = kd;
        slot0Configs.kS = kf;
        m_controller.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kp; 
        slot0Configs.kI = ki;
        slot0Configs.kD = kd;
        m_controller.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void resetIAccum() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetIAccum'");
    }

    @Override
    public void activate(double setpoint, ControlType type) {
        switch (type) {
            case kPos:
                double posConversionFactor = m_everController.getPosConversionFactor();
                m_controller.setControl(new PositionVoltage(setpoint / posConversionFactor).withSlot(0));
                break;
            case kVel:
                double velConversionFactor = m_everController.getVelConversionFactor();
                m_controller.setControl(new VelocityVoltage(setpoint / velConversionFactor).withSlot(0));
                break;    
            default:
                break;
        }
    }

    @Override
    public void stop() {
        m_controller.stopMotor();
    }

    
}
