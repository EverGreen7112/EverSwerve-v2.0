package frc.robot.Subsystems;

import frc.robot.Utils.EverKit.Periodic;

public class SwerveLocalizator implements Periodic{

    private double x, y;
    private SwerveOdometer m_odometer;
    private int xj;

    public SwerveLocalizator(){
        m_odometer = SwerveOdometer.getInstance();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }


}
