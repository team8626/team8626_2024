// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class ShooterSubsystem extends SubsystemBase implements ImplementDashboard {
  private CANSparkMax m_motor1 = new CANSparkMax(ShooterConstants.kCANMotor1, MotorType.kBrushless);
  private CANSparkMax m_motor2 = new CANSparkMax(ShooterConstants.kCANMotor2, MotorType.kBrushless);
  private SparkPIDController m_pidController1 = m_motor1.getPIDController();
  private SparkPIDController m_pidController2 = m_motor2.getPIDController();
  private DigitalInput m_infrared = new DigitalInput(ShooterConstants.kIRSensor);
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

  public boolean isEmpty() {
    // 0 means something is there, 1 means nothing is there
    return m_infrared.get();
  }

  @Override
  public void periodic() {}

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {}

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
