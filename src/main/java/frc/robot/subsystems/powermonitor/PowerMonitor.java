// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.powermonitor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class PowerMonitor implements ImplementDashboard {

  private AnalogInput m_input = new AnalogInput(3);
  private double m_min = 10.0;
  private double m_max = 0.0;
  private double m_currentV = 0;

  public PowerMonitor() {
    m_currentV = m_input.getVoltage() * 2; // Value read is half the voltage
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    m_currentV = m_input.getVoltage() * 2; // Value read is half the voltage.
    m_min = Math.min(m_min, m_currentV);
    m_max = Math.max(m_max, m_currentV);

    SmartDashboard.putNumber("PowerMonitor/Output Voltage (V)", m_currentV);
    SmartDashboard.putNumber("PowerMonitor/Min (V)", m_min);
    SmartDashboard.putNumber("PowerMonitor/Max (V)", m_max);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
