package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Commands.Swerve.DriveByJoysticks.SpeedMode;

public class SwitchSpeedMode extends Command{
    

    private SpeedMode m_mode;
    public SwitchSpeedMode(SpeedMode mode){
        m_mode = mode;
    }

    @Override
    public void initialize() {
        DriveByJoysticks.setSpeedMode(m_mode);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        DriveByJoysticks.setSpeedMode(SpeedMode.kNormal);
    }
}


