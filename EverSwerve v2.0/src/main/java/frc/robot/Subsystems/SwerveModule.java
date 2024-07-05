package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;
import frc.robot.Utils.SensorsAndControllers.EverAbsEncoder;
import frc.robot.Utils.SensorsAndControllers.EverMotorController;
import frc.robot.Utils.SensorsAndControllers.EverMotorController.ControlType;
import frc.robot.Utils.SensorsAndControllers.EverMotorController.IdleMode;

public class SwerveModule extends SubsystemBase {

    // swerve module motor controllers
    private EverMotorController m_driveController;
    private EverMotorController m_steerController;

    // absolute encoder of module
    private EverAbsEncoder m_absEncoder;

    public SwerveModule(EverMotorController driveController, EverMotorController steerController) {
        m_driveController = driveController;
        m_steerController = steerController;
        // configure motor controllers
        m_driveController.restoreFactoryDefaults();
        m_steerController.restoreFactoryDefaults();

        m_driveController.setIdleMode(IdleMode.kCoast);
        m_steerController.setIdleMode(IdleMode.kCoast);

        m_driveController.setConversionFactor(ControlType.kVel,
                SwerveConsts.DRIVE_GEAR_RATIO * SwerveConsts.WHEEL_PERIMETER / 60.0); // convert from rpm without gear
                                                                                      // ratio to m/s with gear ratio
         m_steerController.setConversionFactor(ControlType.kPos, SwerveConsts.STEER_GEAR_RATIO * 360.0); //convert from rotations without gear ratio to degrees with gear ratio                                                                       


        m_driveController.setPidf(SwerveConsts.WHEEL_VELOCITY_KP, SwerveConsts.WHEEL_VELOCITY_KI,
                SwerveConsts.WHEEL_VELOCITY_KD, SwerveConsts.WHEEL_VELOCITY_KF);
        m_steerController.setPidf(SwerveConsts.WHEEL_ANGLE_KP, SwerveConsts.WHEEL_ANGLE_KI,
                SwerveConsts.WHEEL_ANGLE_KD, 0);
    }

    public SwerveModule(EverMotorController driveController, EverMotorController steerController,
            EverAbsEncoder absEncoder, double offset) {
        this(driveController, steerController);
        // configure abs encoder
        m_absEncoder = absEncoder;
        m_absEncoder.setOffset(offset);
        // set steer's integrated encoder pos to the pos of the abs encoder
        m_steerController.setEncoderPos(m_absEncoder.getPos());
    }

     /**
     * set state in cartesian values
     * @param speed - in m/s
     * @param angle - in degrees
     */
    public void setState(double speed, double angle) {
        setState(new Vector2d(speed * Math.cos(Math.toRadians(angle)), speed * Math.sin(Math.toRadians(angle))));
    }

    /**
     * 
     * @param desiredState - 2d vector - magnitude represents the target speed in m/s
     *                     angle represents the target angle
     */
    public void setState(Vector2d desiredState) {
        // rotate vector by 90 because we want 0 degrees to be in the front and not in
        // the right
        desiredState.rotate(Math.toRadians(90));

        // get polar values from desired state vector
        double targetSpeed = desiredState.mag(); // get target speed
        double targetAngle = Math.toDegrees(desiredState.theta()); // convert target angle from radians to degrees

        double currentAngle = getAngle();

        // calculate optimal delta angle
        double optimizedFlippedDeltaTargetAngle = Funcs.closestAngle(currentAngle, targetAngle - 180);
        double optimizedNormalDeltaTargetAngle = Funcs.closestAngle(currentAngle, targetAngle);

        double optimizedDeltaTargetAngle = 0;
        if (Math.abs(optimizedNormalDeltaTargetAngle) > Math.abs(optimizedFlippedDeltaTargetAngle)) {
            optimizedDeltaTargetAngle = optimizedFlippedDeltaTargetAngle;
        } else {
            optimizedDeltaTargetAngle = optimizedNormalDeltaTargetAngle;
        }

        // turn module to target angle
        m_steerController.set(ControlType.kPos, currentAngle + optimizedDeltaTargetAngle);

        // dot product to current state
        targetSpeed *= Math.cos(Math.toRadians(optimizedNormalDeltaTargetAngle));

        // set speed of module at target speed
        m_driveController.set(ControlType.kVel, targetSpeed);
    }

    /**
     * turn module to targetAngle
     * 
     * @param targetAngle in degrees
     */

    public void turnToAngle(double targetAngle) {
        m_steerController.set(ControlType.kPos, targetAngle);
    }

    /**
     * 
     * @return current angle in degrees of module
     */
    public double getAngle(){
        return m_steerController.get(ControlType.kPos);
    }

    public void stopModule(){
        m_driveController.set(0);
        m_steerController.set(0);
    }

}
