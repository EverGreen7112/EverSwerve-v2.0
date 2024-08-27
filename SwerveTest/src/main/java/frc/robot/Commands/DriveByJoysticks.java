package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class DriveByJoysticks extends Command{

    private double JOYSTICK_DEADZONE = 0.2;
    private Supplier<Double> m_speedX, m_speedY, m_rotation;
    private Supplier<Boolean> m_isFieldOriented;
    private boolean m_usesAbsEncoder;
    private double m_currentTime;

    public DriveByJoysticks(Supplier<Double> speedX, Supplier<Double> speedY, Supplier<Double> rotation, Supplier<Boolean> isFieldOriented, boolean usesAbsEncoder){
        addRequirements(Swerve.getInstance());
        m_speedX = speedX;
        m_speedY = speedY;
        m_rotation = rotation;
        m_isFieldOriented = isFieldOriented;
        m_usesAbsEncoder = usesAbsEncoder;
    }

    @Override
    public void initialize() {
        addRequirements(Swerve.getInstance());
        m_currentTime = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void execute() {
        //get values from suppliers
        double speedX = m_speedX.get();
        double speedY = m_speedY.get();
        double rotation = m_rotation.get();
        
        double deltaTime = (System.currentTimeMillis() / 1000.0) - m_currentTime;

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

        //rotate robot according to rotation supplier   
        Swerve.getInstance().rotateBy(SwerveConsts.MAX_ANGULAR_SPEED * rotation * deltaTime);
        //create drive vector
        Vector2d vec = new Vector2d(speedX, -speedY);
        //make sure mag never goes over 1 so driving in all directions will be the same speed
        if(vec.mag() > 1){
            vec.normalise();
        }
        //drive
        Swerve.getInstance().drive(vec, true);
        //update current time
        m_currentTime = System.currentTimeMillis() / 1000.0;
    }
}