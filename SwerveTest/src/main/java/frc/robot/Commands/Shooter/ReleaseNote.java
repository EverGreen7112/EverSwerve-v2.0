package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;

public class ReleaseNote extends Command{
    @Override
    public void initialize() {
        Shooter.getInstance().pushNoteToShoot(0.3);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
        Shooter.getInstance().pushNoteToShoot(0);
    }
}
