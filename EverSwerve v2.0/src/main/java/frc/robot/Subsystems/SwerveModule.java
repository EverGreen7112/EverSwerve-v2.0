package frc.robot.Subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class SwerveModule extends SubsystemBase {

    // swerve module motor controllers
    private EverPIDController m_velocityController;
    private EverMotorController m_driveMotor;
    private EverEncoder m_driveEncoder;
    private EverPIDController m_angleController;
    private EverMotorController m_steerMotor;
    private EverEncoder m_steerEncoder;

    // absolute encoder of module
    private EverAbsEncoder m_absSteerEncoder;

    /**
     * @param velocityController velocity control units of measure should be configured for meters per second(the speed of the wheel)
     * @param angleController angle control units of measure should be configured for degrees(the angle of the wheel)
    */
     public SwerveModule(EverPIDController velocityController, EverMotorController driveMotor,
            EverEncoder driveEncoder, EverPIDController angleController,
            EverMotorController steerMotor, EverEncoder steerEncoder) {
        m_velocityController = velocityController;
        m_driveMotor = driveMotor;
        m_angleController = angleController;
        m_steerMotor = steerMotor;
    }

    /**
     * @param velocityController velocity control units of measure should be configured for meters per second(the speed of the wheel)
     * @param angleController angle control units of measure should be configured for degrees(the angle of the wheel)
     * @param absSteerEncoder absSteerEncoder units of measure should be configured for degrees(absolute of the wheel)
     */
    public SwerveModule(EverPIDController velocityController, EverMotorController driveMotor,
                        EverEncoder driveEncoder, EverPIDController angleController,
                        EverMotorController steerMotor, EverEncoder steerEncoder,
                        EverAbsEncoder absSteerEncoder){
        this(velocityController, driveMotor, driveEncoder,
             angleController, steerMotor, steerEncoder);
        m_absSteerEncoder = absSteerEncoder;
        m_driveEncoder.setPos(absSteerEncoder.getAbsPos());
    }

    /**
     * set state in cartesian values
     * @param speed - in meters per second
     * @param angle - in degrees
     */
    public void setState(double speed, double angle) {
        setState(new Vector2d(speed * Math.cos(Math.toRadians(angle)), speed * Math.sin(Math.toRadians(angle))));
    }

    /**
     * 
     * @param desiredState - 2d vector - magnitude represents the target speed in
     *                     m/s
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
        m_angleController.activate(currentAngle + optimizedDeltaTargetAngle, ControlType.kPos);

        // dot product to current state
        targetSpeed *= Math.cos(Math.toRadians(optimizedNormalDeltaTargetAngle));

        // set speed of module at target speed
        m_velocityController.activate(targetSpeed, ControlType.kVel);
    }

    /**
     * turn module to targetAngle
     * @param targetAngle in degrees
     */
    public void turnToAngle(double targetAngle) {
        m_angleController.activate(targetAngle, ControlType.kPos);
    }

    /**
     * @return current angle in degrees
     */
    public double getAngle() {
        return m_steerEncoder.getPos();
    }

    /**
     * @return current velocity in meters per second
     */
    public double getVelocity() {
        return m_driveEncoder.getVel();
    }

    /**
     * @return module's velocity vector in meters per second
     */
    public Vector2d getVec(){
        double mag = Math.abs(getVelocity());
        double theta = Math.toRadians(getAngle());
        return new Vector2d(mag * Math.cos(theta), mag * Math.sin(theta));
    }

    public void stopModule() {
        m_driveMotor.stop();
        m_steerMotor.stop();
    }

    

}
