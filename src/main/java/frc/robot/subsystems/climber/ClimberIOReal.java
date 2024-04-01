// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Dashboard.DashboardUses;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class ClimberIOReal implements ClimberIO {

  /** Create Motors * */
  private VictorSPX m_motorLeft;

  private VictorSPX m_motorRight;

  private boolean m_enabled = false;

  @AutoLogOutput private double m_appliedPower;

  public ClimberIOReal() {
    // Reset & Initialize Controllers
    m_motorLeft = new VictorSPX(ClimberConstants.CANID_L);
    m_motorRight = new VictorSPX(ClimberConstants.CANID_R);

    m_motorLeft.setInverted(false);
    m_motorRight.setInverted(true);

    m_motorRight.follow(m_motorLeft);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.enabled = m_enabled;
  }

  /**
   * Set Motors Power
   *
   * @newPower new value to be applied [-1.0 ; 1.0]
   */
  @Override
  public void setPower(double newPower) {
    m_appliedPower = newPower;
    m_enabled = (newPower != 0);
    m_motorLeft.set(VictorSPXControlMode.PercentOutput, newPower);
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
