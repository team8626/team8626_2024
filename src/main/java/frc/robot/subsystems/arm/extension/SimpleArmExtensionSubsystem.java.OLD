// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.extension;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.arm.ArmConstants.Extension;

public class SimpleArmExtensionSubsystem extends SubsystemBase implements ImplementDashboard {

  /** Create Motors * */
  private final CANSparkMax m_extensionMotor_L;

  /** Create Sensors * */
  /** Class States * */
  public enum status {
    IDLE(0, "IDLE"),
    RETRACTING(1, "RETRACTING"),
    RETRACTED(2, "RETRACTED"),
    EXTENDING(3, "EXTENDING"),
    EXTENDED(4, "EXTENDED");

    private int m_id;
    private String m_string;

    private status(int id, String string) {
      m_id = id;
      m_string = string;
    }

    public String getString() {
      return m_string;
    }
  }

  private status m_status = status.IDLE;

  /** Creates a new ArmExtensionSubsystem. */
  public SimpleArmExtensionSubsystem() {
    // EXTENSION
    // Motors
    m_extensionMotor_L = new CANSparkMax(Extension.extensionCANID_L, MotorType.kBrushless);

    // Reset & Initialize Controllers
    m_extensionMotor_L.restoreFactoryDefaults();
    m_extensionMotor_L.setInverted(false);
    m_extensionMotor_L.setIdleMode(IdleMode.kBrake);
    m_extensionMotor_L.setSmartCurrentLimit(Extension.kCurrentLimit);

    // Write configuration to Controllers
    m_extensionMotor_L.burnFlash();

    // FORCE DASHBOARD UPDATE (Get values)
    updateDashboard();

    if (RobotBase.isReal()) {
      // TODO: Launch zeroing of the arm
      // this.reset();
    }
  }

  public void retract() {
    m_status = status.RETRACTING;
    System.out.println("[ARM] Retracting");
  }

  public void extend() {
    m_status = status.EXTENDING;
    System.out.println("[ARM] Retracting");
  }

  @Override
  public void periodic() {
    switch (m_status) {
      case RETRACTING:
        if (m_extensionMotor_L.getOutputCurrent() < Extension.kZeroingCurrent) {
          m_extensionMotor_L.set(-0.8);
          System.out.println("[ARM] Retracting");
        } else {
          m_extensionMotor_L.set(0);
          m_status = status.RETRACTED;
          System.out.println("[ARM] Retracted");
        }
        break;

      case EXTENDING:
        if (m_extensionMotor_L.getOutputCurrent() < Extension.kZeroingCurrent) {
          m_extensionMotor_L.set(0.6);
          System.out.println("[ARM] Extending");
        } else {
          m_extensionMotor_L.set(0);
          m_status = status.EXTENDED;
          System.out.println("[ARM] Extended");
        }
        break;

      case RETRACTED:
      case EXTENDED:
      case IDLE:
        break;
    }
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    SmartDashboard.putString("Arm/Extension/Status", m_status.getString());
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }

  private void updateSimValues() {}
}
