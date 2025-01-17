package frc.robot.Commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveConsts;

public class ChangeTeleopSpeedModeCommand extends Command{

    public enum SpeedMode{
        kNormal,
        kTurbo,
        kSlow
    }

    private SpeedMode m_mode;

    public ChangeTeleopSpeedModeCommand(SpeedMode speedMode){
        m_mode = speedMode;
    }

    @Override
    public void initialize() {
        switch (m_mode) {
            case kSlow:
                TeleopDriveCommand.maxSpeed = SwerveConsts.MAX_SLOW_DRIVE_SPEED;    
                break;
            case kTurbo:
                TeleopDriveCommand.maxSpeed = SwerveConsts.MAX_TURBO_DRIVE_SPEED;    
                break;
            case kNormal:
                TeleopDriveCommand.maxSpeed = SwerveConsts.MAX_NORMAL_DRIVE_SPEED;
                break;
            default:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    
}
