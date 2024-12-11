package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAngleController;

public class LockSwerveAngleCommand extends Command{
    
    @Override
    public void initialize() {
        SwerveAngleController.getInstance().start(Swerve.getInstance().getGyroOrientedAngle());
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
