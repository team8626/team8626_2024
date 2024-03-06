// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.preset;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Presets.Preset;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.preset.Presets.Preset;

public class PresetManager implements ImplementDashboard {
  private Preset m_preset;

  public PresetManager() {
    m_preset = Preset.kShootSubwoofer;
  }

  public void set(Preset newPreset) {
    m_preset = newPreset;
  }

  public Preset get() {
    return m_preset;
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    SmartDashboard.putString("Arm/Preset", m_preset.getString());
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.LONG_INTERVAL;
  }
}
