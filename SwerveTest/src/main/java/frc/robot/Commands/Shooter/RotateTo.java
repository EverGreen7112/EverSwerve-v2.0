package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class RotateTo extends Command{

    private double m_targetAngle;

    public RotateTo(double targetAngle){
        m_targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        addRequirements(Shooter.getInstance());
        Shooter.getInstance().turnToAngle(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
