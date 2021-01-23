package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.common.AbsoluteEncoder;

public class SwerveWheel {
    private CANSparkMax m_drivemotor;
    private CANSparkMax m_turnmotor;
    private AbsoluteEncoder m_encoder;
    private Translation2d m_location;
    private final PIDController m_turningPID = new PIDController(0.3, 0, 0);

    public SwerveWheel(CANSparkMax drivemotor, CANSparkMax turnmotor, AbsoluteEncoder encoder, double X, double Y) {
        m_drivemotor = drivemotor;
        m_turnmotor = turnmotor;
        m_encoder = encoder;
        m_location = new Translation2d(X, Y);
        m_drivemotor.setSmartCurrentLimit(35, 60, 150);
        m_turnmotor.setSmartCurrentLimit(17, 30, 75);

        m_turningPID.enableContinuousInput(0, 2 * Math.PI);
    }

    public void setDesiredState(SwerveModuleState target){
        m_drivemotor.set(target.speedMetersPerSecond/Constants.Swerve.MaxSpeedOfWheel);
        double pidOutput = m_turningPID.calculate(m_encoder.getRadians(), target.angle.getRadians());

        //capping the pid output at a range between -1 to 1
        if (pidOutput > 1) {
            pidOutput = 1;
        } else if (pidOutput < -1) {
            pidOutput = -1;
        }
        m_turnmotor.set(pidOutput);
    }
}