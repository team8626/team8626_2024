// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import frc.robot.subsystems.Dashboard.ImplementDashboard;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO extends ImplementDashboard {

  @AutoLog
  public class ClimberIOInputs {
    public boolean enabled;
    public double appliedPower;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}
  ;

  public default void setPower(double newPower) {}
  ;
}
