// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  ClimberIO m_io = new ClimberIOReal();
  ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new ArmExtensionSubsystem. */
  public ClimberSubsystem(ClimberIO io) {
    m_io = io;
  }

  /**
   * Set Motors Power
   *
   * @newPower new value to be applied [-1.0 ; 1.0]
   */
  public void setPower(double newPower) {
    m_io.setPower(newPower);
  }

  public ClimberIO getIO() {
    return m_io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.processInputs("ClimberSubsystem", inputs);
  }
}
