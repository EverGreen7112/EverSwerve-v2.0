package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;


public class IntakeNote extends Command{

    public IntakeNote(){
        addRequirements(Intake.getInstance());
    }

    @Override
    public void initialize() {
        Intake.getInstance().emitNote();
    }

    @Override
    public void execute(){}

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().stop();
    }

    
}
