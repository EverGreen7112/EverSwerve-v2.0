package frc.robot.Commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class TeleopDriveCommand extends Command{
    
    private final double DEADZONE = 0.2;
    public static double maxSpeed;
    private Supplier<Double> m_xSpeedInput;
    private Supplier<Double> m_ySpeedInput;
    private Supplier<Double> m_angularVelocityInput;
    
    public TeleopDriveCommand(Supplier<Double> xSpeedInput, Supplier<Double> ySpeedInput, Supplier<Double> angularVelocityInput){
        m_xSpeedInput = xSpeedInput;
        m_ySpeedInput = ySpeedInput;
        m_angularVelocityInput = angularVelocityInput;
        maxSpeed = SwerveConsts.MAX_NORMAL_DRIVE_SPEED;
    }

    @Override
    public void execute() {
        
        double speedX = m_xSpeedInput.get();
        double speedY = m_ySpeedInput.get();
        double angularVel = m_angularVelocityInput.get();

        if(Math.abs(speedX) < DEADZONE)
            speedX = 0;
        if(Math.abs(speedY) < DEADZONE)
            speedY = 0; 
        if(Math.abs(angularVel) < DEADZONE)
            angularVel = 0;

        angularVel = Funcs.roundAfterDecimalPoint(angularVel, 2);
        speedX = Funcs.roundAfterDecimalPoint(speedX, 2);
        speedY = Funcs.roundAfterDecimalPoint(speedY, 2);
    
         //create drive vector
        Vector2d vec = new Vector2d(-speedX * maxSpeed, speedY * maxSpeed);
        
        //make sure mag never goes over maxDriveSpeed so driving in all directions will be the same speed
        if(vec.mag() > maxSpeed){
            vec.normalise();
            vec.mul(maxSpeed);
        }

        //drive
        Swerve.getInstance().drive(Funcs.convertFromStandardAxesToWpilibs(vec), true, -angularVel * SwerveConsts.MAX_ANGULAR_SPEED);       
            
    }


}   
