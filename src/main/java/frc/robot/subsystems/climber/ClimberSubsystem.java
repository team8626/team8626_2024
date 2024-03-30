// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class ClimberSubsystem extends SubsystemBase implements ImplementDashboard {

  /** Create Motors * */
  private VictorSPX m_motorLeft = null;

  private VictorSPX m_motorRight = null;
  private boolean m_usingWindowMotors = false;

  private boolean m_enabled = false;
  private boolean m_brakeEnabled = false;

  private Servo m_brakeServo;

  /** Creates a new ArmExtensionSubsystem. */
  public ClimberSubsystem() {
    // Reset & Initialize Controllers
    if (m_usingWindowMotors) {
      m_motorLeft = new VictorSPX(ClimberConstants.CANID_L);
      m_motorRight = new VictorSPX(ClimberConstants.CANID_R);

      m_motorLeft.setInverted(false);
      m_motorRight.setInverted(true);

      m_motorRight.follow(m_motorLeft);
    }

    // Usign Servo Motor for braking
    m_brakeServo = new Servo(ClimberConstants.kServoPort);
    this.setBrake(false);
  }

  /**
   * Set Motors Power
   *
   * @newPower new value to be applied [-1.0 ; 1.0]
   */
  public void setPower(double newPower) {
    if (m_usingWindowMotors) {
      if (newPower != 0) {
        m_enabled = true;
      } else {
        m_enabled = false;
      }
      m_motorLeft.set(VictorSPXControlMode.PercentOutput, newPower);
    }
  }

  public void setBrake(boolean newState) {
    m_brakeEnabled = newState;
    if (m_brakeEnabled) {
      m_brakeServo.setAngle(180);
    } else {
      m_brakeServo.setAngle(0);
    }
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Climber/ENABLED", m_enabled);
    SmartDashboard.putBoolean("Climber/Brake", m_brakeEnabled);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
