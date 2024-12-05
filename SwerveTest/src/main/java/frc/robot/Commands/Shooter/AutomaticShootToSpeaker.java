package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class AutomaticShootToSpeaker extends Command{

     @Override
    public void initialize() {
        
        Shooter.getInstance().shoot();
    }

    @Override
    public void execute() {
        Shooter.getInstance().autoAim();
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Shooter.getInstance().stopShoot();;
    }
    
}
