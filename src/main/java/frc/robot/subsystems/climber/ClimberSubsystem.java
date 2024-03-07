// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class ClimberSubsystem extends SubsystemBase implements ImplementDashboard {

  /** Create Motors * */
  private final VictorSPX m_motorLeft;

  private final VictorSPX m_motorRight;

  private boolean m_enabled = false;

  private boolean m_isClimberActive = false;

  /** Creates a new ArmExtensionSubsystem. */
  public ClimberSubsystem() {
    // Reset & Initialize Controllers
    m_motorLeft = new VictorSPX(ClimberConstants.CANID_L);
    m_motorRight = new VictorSPX(ClimberConstants.CANID_R);

    m_motorLeft.setInverted(false);
    m_motorRight.setInverted(true);

    m_motorRight.follow(m_motorLeft);
  }

  /**
   * Set Motors Power
   *
   * @newPower new value to be applied [-1.0 ; 1.0]
   */
  public void setPower(double newPower) {
    if (newPower != 0) {
      m_enabled = true;
    } else {
      m_enabled = false;
    }
    m_motorLeft.set(VictorSPXControlMode.PercentOutput, newPower);
  }

  public void toggleClimberActive() {
    m_isClimberActive = !m_isClimberActive;
  }

  public boolean isClimberActive() {
    return m_isClimberActive;
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Climber/ENABLED", m_enabled);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
