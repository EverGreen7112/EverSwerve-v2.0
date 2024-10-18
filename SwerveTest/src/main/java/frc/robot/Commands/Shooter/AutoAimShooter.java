package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class AutoAimShooter extends Command{
    @Override
    public void initialize() {
        Shooter.getInstance().turnToAngle(Shooter.getInstance().getShooterAngleToSpeaker());
    }

    @Override
    public boolean isFinished() {
        return true; 
    }
}
