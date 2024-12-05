package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShootToAmp extends Command {

    public ShootToAmp(){}

    public void initialize(){
        Shooter.getInstance().turnToAmp();;
        Shooter.getInstance().shootToAmp();
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();
    }
    
}
