// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates.IntakeStatus;

/** Add your docs here. */
public interface IntakeIO extends ImplementDashboard {

  @AutoLog
  public class IntakeIOInputs {
    public double speed;
    public boolean enabled;
    public IntakeStatus status;
    public boolean bottomSensor;
    public boolean exitSensor;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
  ;
  public default void setSpeed(double speed){};
  public default void start(){};
  public default void start(double speed){};
  public default void stop(){};
  public default void setStatus(IntakeStatus status){};
  public boolean isFull();
  public boolean isEmpty();
  public boolean limitReached();
}