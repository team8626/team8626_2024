// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;

public class IntakeSubsystem extends SubsystemBase {

private CANSparkMax m_motorLeft = new CANSparkMax(8, MotorType.kBrushless);
private CANSparkMax m_motorRight = new CANSparkMax(9, MotorType.kBrushless);

  public IntakeSubsystem() {
    m_motorLeft.setInverted(false);
    m_motorRight.setInverted(false);
  }

  public void setMotors(double output) {
    m_motorLeft.set(output);
    m_motorRight.set(output);
  }
}
