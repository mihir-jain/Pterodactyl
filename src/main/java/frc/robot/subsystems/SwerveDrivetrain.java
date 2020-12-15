/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.AbsoluteEncoder;

public class SwerveDrivetrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  PIDController pidControl;
  private CANSparkMax frontLeftDrive;
  private CANSparkMax frontLeftTurn;
  private CANSparkMax frontRightDrive;
  private CANSparkMax frontRightTurn;
  private CANSparkMax backLeftDrive;
  private CANSparkMax backLeftTurn;
  private CANSparkMax backRightDrive;
  private CANSparkMax backRightTurn;
  private AbsoluteEncoder frontLeftEncoder;
  private AbsoluteEncoder frontRightEncoder;
  private AbsoluteEncoder backLeftEncoder;
  private AbsoluteEncoder backRightEncoder;
  private SwerveDriveKinematics kinematics;
  private AHRS m_gyro;

  private double _x;
  private double _y;
  private double _theta;
  private boolean _fieldRelative;
  private boolean _isTurning;

  public SwerveDrivetrain() {

    pidControl = new PIDController(0, 0, 0);
    pidControl.enableContinuousInput(0, 360);
    pidControl.setTolerance(10);
  }

  public void move(double x, double y, double theta, boolean fieldRelative) {
    _x = x;
    _y = y;
    _theta = theta;
    _fieldRelative = fieldRelative;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
    SwerveModuleState[] serveModuleStates;
    if (_fieldRelative) {
      serveModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(_x, _y, _theta, new Rotation2d(2 * Math.PI - Math.toRadians(((((m_gyro.getAngle() % 360) + 360) % 360))))));
    } else {
      serveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(_x, _y, _theta));
    }
  }
}
