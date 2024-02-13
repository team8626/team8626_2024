// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class IntakeSubsystem extends SubsystemBase implements ImplementDashboard {
  // private CANSparkMax m_motor1 = new CANSparkMax(IntakeConstants.kCANMotor1,
  // MotorType.kBrushless);
  private VictorSPX m_motor1 = new VictorSPX(IntakeConstants.kCANMotor1);
  private VictorSPX m_motor2 = new VictorSPX(IntakeConstants.kCANMotor2);
  private DigitalInput m_infrared1 = new DigitalInput(IntakeConstants.kIRSensor1);
  private DigitalInput m_infrared2 = new DigitalInput(IntakeConstants.kIRSensor2);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void setMotors(double speed) {
    m_motor1.set(VictorSPXControlMode.PercentOutput, speed);
    m_motor2.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public boolean isFull() {
    // 0 means something is there, 1 means nothing is there
    return m_infrared1.get();
  }

  public boolean limitReached() {
    return !m_infrared2.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limit Switch", limitReached());
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Intake Loaded", !m_infrared1.get());
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
