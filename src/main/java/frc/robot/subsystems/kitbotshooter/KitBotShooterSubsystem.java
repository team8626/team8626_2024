// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitbotshooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;

public class KitBotShooterSubsystem extends SubsystemBase {

  private static CANSparkMax m_motorLeft = new CANSparkMax(ClimberConstants.kClimberMotorLeftPort, MotorType.kBrushless);
  private static CANSparkMax m_motorRight = new CANSparkMax(ClimberConstants.kClimberMotorRightPort, MotorType.kBrushless);

  public KitBotShooterSubsystem() {
    m_motorRight.setInverted(true);
  }

public void setMotors(double output) {
  m_motorLeft.set(output);
  m_motorRight.set(output);
}

  @Override
  public void periodic() {
  }
}
