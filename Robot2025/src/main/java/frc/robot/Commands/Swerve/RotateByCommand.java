package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;

public class RotateByCommand extends Command{
    private double m_targetAngle;

    public RotateByCommand(double targetAngle){
        m_targetAngle = Swerve.getInstance().getGyroOrientedAngle() + targetAngle;
    }

    @Override
    public void initialize() {
        SwerveAngleController.getInstance().start(m_targetAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveAngleController.getInstance().stop();
    }
}
