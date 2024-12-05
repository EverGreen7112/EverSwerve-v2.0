package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climb.Climb;

public class RetractLeftClimber extends Command{
    @Override
    public void execute(){
        Climb.getInstance().retractLeft();
    }

    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Climb.getInstance().stopLeft();
    }
    
}
