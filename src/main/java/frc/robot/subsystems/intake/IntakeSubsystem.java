// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class IntakeSubsystem extends SubsystemBase implements ImplementDashboard {
  private CANSparkMax m_motor1 = new CANSparkMax(IntakeConstants.kCANMotor1, MotorType.kBrushless);
  private CANSparkMax m_motor2 = new CANSparkMax(IntakeConstants.kCANMotor2, MotorType.kBrushless);
  private DigitalInput m_infrared = new DigitalInput(IntakeConstants.kIRSensor);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_motor2.setInverted(true);
  }

  public void setMotors(double speed){
    m_motor1.set(speed);
    m_motor2.set(speed);
  }
  public  boolean getSensor(){
    // 0 means something is there, 1 means nothing is there
    return m_infrared.get();
  }
  @Override
  public void periodic(){}

  @Override
  public void initDashboard() {
  }

  @Override
  public void updateDashboard() {
  // SmartDashboard.putNumber("Intake State", m_infrared.get());
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
