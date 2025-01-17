package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;

public class RotateToCommand extends Command{
    
    private double m_targetAngle;

    public RotateToCommand(double targetAngle){
        m_targetAngle = targetAngle;
    }

    @Override
    public void initialize() {
        SwerveAngleController.getInstance().start(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
