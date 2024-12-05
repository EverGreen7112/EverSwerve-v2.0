package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShootToSpeaker extends Command {

    public ShootToSpeaker(){}

    public void initialize(){
        Shooter.getInstance().turnTo(130);
        Shooter.getInstance().shoot();
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();
    }
    
}
