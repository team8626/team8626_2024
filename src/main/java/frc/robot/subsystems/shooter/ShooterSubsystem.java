// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class ShooterSubsystem extends SubsystemBase implements ImplementDashboard {
  private CANSparkMax m_motor1 = new CANSparkMax(ShooterConstants.kCANMotor1, MotorType.kBrushless);
  private CANSparkMax m_motor2 = new CANSparkMax(ShooterConstants.kCANMotor2, MotorType.kBrushless);
  private SparkPIDController m_pidController1 = m_motor1.getPIDController();
  private SparkPIDController m_pidController2 = m_motor2.getPIDController();

  private double m_desiredRPM_Top = 0;
  private double m_desiredRPM_Bottom = 0;
  private double m_currentRPM_Top = 0;
  private double m_currentRPM_Bottom = 0;

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    double kP = 6e-5;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0.000015;
    double kMaxOutput = 1;
    double kMinOutput = -1;

    // set PID coefficients
    m_pidController1.setP(kP);
    m_pidController1.setI(kI);
    m_pidController1.setD(kD);
    m_pidController1.setIZone(kIz);
    m_pidController1.setFF(kFF);
    m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

    m_pidController2.setP(kP);
    m_pidController2.setI(kI);
    m_pidController2.setD(kD);
    m_pidController2.setIZone(kIz);
    m_pidController2.setFF(kFF);
    m_pidController2.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void setMotors(double speed) {
    m_pidController1.setReference(speed, CANSparkMax.ControlType.kVelocity);
    m_pidController2.setReference(speed, CANSparkMax.ControlType.kVelocity);
    // m_motor1.set(speed);
    // m_motor2.set(speed);
  }

  public void setRPM(double speed_top, double speed_bottom) {
    m_desiredRPM_Top = speed_top;
    m_desiredRPM_Bottom = speed_bottom;

    // Force Dashboard Update
    initDashboard();
  }

  @Override
  public void periodic() {
    m_pidController1.setReference(m_desiredRPM_Top, CANSparkMax.ControlType.kVelocity);
    m_pidController2.setReference(m_desiredRPM_Bottom, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void initDashboard() {
    SmartDashboard.putNumber("Shooter/DesiredRPM_TOP", m_desiredRPM_Top);
    SmartDashboard.putNumber("Shooter/DesiredRPM_BOTTOM", m_desiredRPM_Bottom);

    SmartDashboard.putNumber("Shooter/CurrentRPM_TOP", m_currentRPM_Top);
    SmartDashboard.putNumber("Shooter/CurrentRPM_BOTTOM", m_currentRPM_Bottom);
  }

  @Override
  public void updateDashboard() {
    double newRPM_Top = SmartDashboard.getNumber("Shooter/DesiredRPM_TOP", m_desiredRPM_Top);
    double newRPM_Bottom =
        SmartDashboard.getNumber("Shooter/DesiredRPM_BOTTOM", m_desiredRPM_Bottom);

    if (newRPM_Top != m_desiredRPM_Top) {
      m_desiredRPM_Top = newRPM_Top;
    }
    if (newRPM_Bottom != m_desiredRPM_Bottom) {
      m_desiredRPM_Bottom = newRPM_Bottom;
    }

    SmartDashboard.putNumber("Shooter/DesiredRPM_TOP", m_desiredRPM_Top);
    SmartDashboard.putNumber("Shooter/DesiredRPM_BOTTOM", m_desiredRPM_Bottom);

    SmartDashboard.putNumber("Shooter/CurrentRPM_TOP", m_currentRPM_Top);
    SmartDashboard.putNumber("Shooter/CurrentRPM_BOTTOM", m_currentRPM_Bottom);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
