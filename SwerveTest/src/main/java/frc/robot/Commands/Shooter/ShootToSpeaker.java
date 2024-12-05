package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class ShootToSpeaker extends Command {

    public ShootToSpeaker(){}

    public void initialize(){
        addRequirements(Shooter.getInstance());
        Shooter.getInstance().turnTo(114); //114
        Shooter.getInstance().shoot(1, 1);
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();
    }
    
}
