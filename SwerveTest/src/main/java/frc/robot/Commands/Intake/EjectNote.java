package frc.robot.Commands.Intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConsts;


public class EjectNote extends Command {

    private double m_speed;

    public EjectNote(double speed){
        this.m_speed = speed;
    }

    @Override
    public void initialize(){
        addRequirements(Intake.getInstance());
        addRequirements(Shooter.getInstance());
        Shooter.getInstance().turnToAngle(ShooterConsts.AIM_MOTOR_MIN_ANGLE);
        Shooter.getInstance().pullNote(m_speed);
        Intake.getInstance().eject();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        Intake.getInstance().stop();
        Shooter.getInstance().pullNote(0);
    }
}