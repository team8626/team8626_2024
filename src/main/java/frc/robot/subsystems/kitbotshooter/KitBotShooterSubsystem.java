// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitbotshooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;


public class KitBotShooterSubsystem extends SubsystemBase {

  private static VictorSP m_motorLeft = new VictorSP(7);
 private static VictorSP m_motorRight = new VictorSP(23);


  public KitBotShooterSubsystem() {
    m_motorRight.setInverted(false);
  }

public void setMotors(double output) {
  m_motorLeft.set(output);
  m_motorRight.set(output);
}

  @Override
  public void periodic() {
  }
}
