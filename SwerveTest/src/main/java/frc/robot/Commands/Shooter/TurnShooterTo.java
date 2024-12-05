package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class TurnShooterTo extends Command {
    private double m_targetAngle;

    public TurnShooterTo(double targetAngle){
        m_targetAngle = targetAngle;
    }

    public void initialize(){
        Shooter.getInstance().turnTo(m_targetAngle);
    }   

    @Override
    public boolean isFinished() {
        return true;
    }
    
}