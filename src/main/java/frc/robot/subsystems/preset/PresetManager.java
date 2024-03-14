// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.preset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Presets.Preset;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.preset.Presets.Preset;

public class PresetManager implements ImplementDashboard {
  private Preset m_preset;
  private Pose2d m_closestPose;

  StructPublisher<Pose2d> m_publisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Preset/Pose2d", Pose2d.struct)
          .publish();

  public PresetManager() {
    m_preset = Preset.kShootSubwoofer;
  }

  public void set(Preset newPreset) {
    m_preset = newPreset;
  }

  public Preset get() {
    return m_preset;
  }

  public Pose2d getPose() {
    return m_preset.getPose();
  }

  public Pose2d getNearest(Pose2d newPose) {
    if (m_preset.getPoses() != null) {
      m_closestPose = newPose.nearest(m_preset.getPoses());
    } else {
      m_closestPose = m_preset.getPose();
    }
    return m_closestPose;
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    m_publisher.set(m_preset.getPose());
    m_publisher.set(m_closestPose);

    SmartDashboard.putString("Preset/Preset", m_preset.getString());
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
