// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitbotshooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;


public class KitBotShooterSubsystem extends SubsystemBase {

  private static VictorSPX m_motorFront = new VictorSPX(7);
 private static VictorSPX m_motorBack = new VictorSPX(23);


  public KitBotShooterSubsystem() {
  }

public void setShoot(double output) {
  m_motorFront.set(ControlMode.PercentOutput, output);
}
public void setMove(double output) {
  m_motorBack.set(ControlMode.PercentOutput,output);
}

  @Override
  public void periodic() {
  }
}
