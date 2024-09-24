package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class DriveByJoysticks extends Command{

    private double JOYSTICK_DEADZONE = 0.2;
    private Supplier<Double> m_speedX, m_speedY, m_rotation;

    public DriveByJoysticks(Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> rotation){
        addRequirements(Swerve.getInstance());
        m_speedX = speedX;
        m_speedY = speedY;
        m_rotation = rotation;
    }

    @Override
    public void initialize() {
        addRequirements(Swerve.getInstance());
    }

    @Override
    public void execute() {
        //get values from suppliers
        double speedX = m_speedX.get();
        double speedY = m_speedY.get();
        double rotation = m_rotation.get();
        
        //apply deadzone on supplier values
        if(Math.abs(speedX) < JOYSTICK_DEADZONE)
            speedX = 0;
        if(Math.abs(speedY) < JOYSTICK_DEADZONE)
            speedY = 0; 
        if(Math.abs(rotation) < JOYSTICK_DEADZONE)
            rotation = 0;

        //round values
        rotation = Funcs.roundAfterDecimalPoint(rotation, 2);
        speedX = Funcs.roundAfterDecimalPoint(speedX, 2);
        speedY = Funcs.roundAfterDecimalPoint(speedY, 2);

        //create drive vector
        Vector2d vec = new Vector2d(-speedX * SwerveConsts.MAX_DRIVE_SPEED.get(), speedY * SwerveConsts.MAX_DRIVE_SPEED.get());
        
        //make sure mag never goes over 1 so driving in all directions will be the same speed
        if(vec.mag() > Swerve.MAX_DRIVE_SPEED.get()){
            vec.normalise();
            vec.mul(Swerve.MAX_DRIVE_SPEED.get());
        }

        //drive
        Swerve.getInstance().drive(Funcs.convertFromStandardAxesToWpilibs(vec), true, -rotation * SwerveConsts.MAX_ANGULAR_SPEED.get());
        
    }
}