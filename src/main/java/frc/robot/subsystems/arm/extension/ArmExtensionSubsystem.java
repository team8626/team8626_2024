// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.extension;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmExtensionSubsystem extends SubsystemBase {
  ArmExtensionIO m_io;
  ArmExtensionInputsAutoLogged inputs = new ArmExtensionInputsAutoLogged();

  public ArmExtensionSubsystem(ArmExtensionIO io) {
    m_io = io;
  }

  /**
   * Set Arm Length to a specific extension in Inches.
   *
   * @param newLengthInches
   */
  public void setLengthInches(double newLengthInches) {
    m_io.setLengthInches(newLengthInches);
  }

  public void reset() {}

  public boolean atSetpoint() {
    return m_io.atSetpoint();
  }

  public double getExtInches() {
    return m_io.getExtInches();
  }

  public ArmExtensionIO getIO() {
    return m_io;
  }

  @Override
  public void periodic() {
    m_io.periodic();
    m_io.updateInputs(inputs);
    Logger.processInputs("ArmExtensionSubsystem", inputs);
  }
}
