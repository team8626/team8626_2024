// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.extension;

import frc.robot.subsystems.Dashboard.ImplementDashboard;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmExtensionIO extends ImplementDashboard {

  @AutoLog
  public class ArmExtensionInputs {
    public double currentExtInches = 0;
    public boolean armIsResetting = false;
    public boolean armZeroed = false;
    public boolean atSetpoint;
    public double leftMotorAmperage;
    public double m_extensionP;
    public double m_extensionFeedForward;
    public double m_extensionD;
  }

  public default void updateInputs(ArmExtensionInputs inputs) {}
  ;

  /**
   * Set Arm Length to a specific extension in Inches.
   *
   * @param newLengthInches
   */
  public default void setLengthInches(double newLengthInches) {}

  public default void reset() {}

  public default boolean atSetpoint() {
    return false;
  }

  public default double getExtInches() {
    return ExtConstants.kMinExtInches;
  }

  public default void periodic() {}
}
