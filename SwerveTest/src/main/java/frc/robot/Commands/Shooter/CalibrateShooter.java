package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class CalibrateShooter extends Command{
    public CalibrateShooter(){
    }

    @Override
    public void initialize() {
        Shooter.getInstance().shoot();
    }

    @Override
    public void execute() {
        double targetAngle = SmartDashboard.getNumber("target angle", 90);
        Shooter.getInstance().turnTo(targetAngle);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();
    }

}
