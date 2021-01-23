/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.AbsoluteEncoder;

public class SwerveDrivetrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  PIDController pidControl;
  private SwerveWheel m_frontLeft;
  private SwerveWheel m_backRight;
  private SwerveWheel m_backLeft;
  private SwerveWheel m_frontRight;

  private CANSparkMax m_frontLeftDrive;
  private CANSparkMax m_frontLeftTurn;
  private CANSparkMax m_frontRightDrive;
  private CANSparkMax m_frontRightTurn;
  private CANSparkMax m_backLeftDrive;
  private CANSparkMax m_backLeftTurn;
  private CANSparkMax m_backRightDrive;
  private CANSparkMax m_backRightTurn;
  private AbsoluteEncoder m_frontLeftEncoder;
  private AbsoluteEncoder m_frontRightEncoder;
  private AbsoluteEncoder m_backLeftEncoder;
  private AbsoluteEncoder m_backRightEncoder;
  private SwerveDriveKinematics kinematics;
  private Gyro m_gyro;

  private double _x;
  private double _y;
  private double _theta;
  private boolean _fieldRelative;
  private boolean _isTurning;

  public SwerveDrivetrain() {

    pidControl = new PIDController(1, 0, 0);
    pidControl.enableContinuousInput(0, 360);
    pidControl.setTolerance(10);

    double widthOffset = Constants.Physical.widthInMeters / 2;
    double lengthOffset = Constants.Physical.lengthInMeters / 2;

    m_frontLeftDrive = new CANSparkMax(Constants.Drivetrain.frontLeftDriveMotorPort, MotorType.kBrushless);
    m_frontLeftDrive.setInverted(true);
    m_frontLeftTurn = new CANSparkMax(Constants.Drivetrain.frontLeftTurnMotorPort, MotorType.kBrushless);
    m_frontLeftEncoder = new AbsoluteEncoder(Constants.Drivetrain.frontLeftAbsoluteEncoder, 2.254991, true);

    m_frontLeft = new SwerveWheel(m_frontLeftDrive, m_frontLeftTurn, m_frontLeftEncoder, widthOffset, lengthOffset);

    m_frontRightDrive = new CANSparkMax(Constants.Drivetrain.frontRightDriveMotorPort, MotorType.kBrushless);
    m_frontRightDrive.setInverted(false);
    m_frontRightTurn = new CANSparkMax(Constants.Drivetrain.frontRightTurnMotorPort, MotorType.kBrushless);
    m_frontRightEncoder = new AbsoluteEncoder(Constants.Drivetrain.frontRightAbsoluteEncoder, 2.466641, true);

    m_frontRight = new SwerveWheel(m_frontRightDrive, m_frontRightTurn, m_frontRightEncoder, widthOffset, -lengthOffset);

    m_backLeftDrive = new CANSparkMax(Constants.Drivetrain.rearLeftDriveMotorPort, MotorType.kBrushless);
    m_backLeftDrive.setInverted(true);
    m_backLeftTurn = new CANSparkMax(Constants.Drivetrain.rearLeftTurnMotorPort, MotorType.kBrushless);
    m_backLeftEncoder = new AbsoluteEncoder(Constants.Drivetrain.rearLeftAbsoluteEncoder, .279, true);
    m_backLeft = new SwerveWheel(m_backLeftDrive, m_backLeftTurn, m_backLeftEncoder, -widthOffset, lengthOffset);

    m_backRightDrive = new CANSparkMax(Constants.Drivetrain.rearRightDriveMotorPort, MotorType.kBrushless);
    m_backRightDrive.setInverted(false);
    m_backRightTurn = new CANSparkMax(Constants.Drivetrain.rearRightturnMotorPort, MotorType.kBrushless);
    m_backRightEncoder = new AbsoluteEncoder(Constants.Drivetrain.rearRightAbsoluteEncoder, 3.925, true);
    m_backRight = new SwerveWheel(m_backRightDrive, m_backRightTurn, m_backRightEncoder, -widthOffset, -lengthOffset);
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

    m_frontLeft.setDesiredState(serveModuleStates[0]);
    m_frontRight.setDesiredState(serveModuleStates[1]);
    m_backLeft.setDesiredState(serveModuleStates[2]);
    m_backRight.setDesiredState(serveModuleStates[3]);
  }
}
