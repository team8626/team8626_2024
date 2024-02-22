// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.extension;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Presets.Preset;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.arm.ArmConstants.Extension;

public class AnotherArmExtensionSubsystem extends SubsystemBase implements ImplementDashboard {

  /** Create Motors * */
  private final CANSparkMax m_extensionMotor_L;

  /** Create Sensors * */
  private final RelativeEncoder m_encoder;

  /** Class States * */
  private double m_desiredExtensionInches = 0;

  private double m_currentExtInches = 0;
  private status m_status;

  private enum status {
    IDLE(-1, "IDLE"),
    UNKNOWN(0, "UNKNOWN"),
    RESSETING(1, "RESSETING"),
    RETRACTING(2, "RETRACTING"),
    EXTENDING(3, "EXTENDING");

    private String m_string;

    private status(int id, String string) {
      m_string = string;
    }

    public String getString() {
      return m_string;
    }
  }

  /** Creates a new ArmExtensionSubsystem. */
  public AnotherArmExtensionSubsystem() {
    // EXTENSION
    // Motors
    m_extensionMotor_L = new CANSparkMax(Extension.extensionCANID_L, MotorType.kBrushless);

    // Reset & Initialize Controllers
    m_extensionMotor_L.restoreFactoryDefaults();

    // Setup encoders.
    m_encoder = m_extensionMotor_L.getEncoder();

    // Apply position and velocity conversion factors for the turning encoder.
    // We want these in degrees and degrees per second
    m_extensionMotor_L.setInverted(false);
    m_encoder.setPositionConversionFactor(Extension.kExtensionEncoderPositionFactorDeg);
    m_encoder.setVelocityConversionFactor(Extension.kExtensionEncoderVelocityFactorDeg);

    // Idle Modes and max current
    m_extensionMotor_L.setIdleMode(IdleMode.kBrake);
    m_extensionMotor_L.setSmartCurrentLimit(Extension.kCurrentLimit);

    // Write configuration to Controllers
    m_extensionMotor_L.burnFlash();

    // FORCE DASHBOARD UPDATE (Get values)
    updateDashboard();

    if (RobotBase.isReal()) {
      // TODO: Launch zeroing of the arm
      // this.reset();
      m_status = status.UNKNOWN;
    } else if (RobotBase.isSimulation()) {
      m_desiredExtensionInches = Preset.kStart.getExtInches();
      m_currentExtInches = m_desiredExtensionInches;
    }
  }

  /**
   * Set Arm Length to a specific extension in Inches.
   *
   * @param newLengthInches
   */
  public void setLengthInches(double newLengthInches) {
    if (m_status != status.UNKNOWN) {
      m_desiredExtensionInches = newLengthInches; // Value Clamped in periodic()
      System.out.printf("[ARM] New Length: %f\n", m_desiredExtensionInches);

      if (m_desiredExtensionInches > m_currentExtInches) {
        m_status = status.EXTENDING;
      } else if (m_desiredExtensionInches < m_currentExtInches) {
        m_status = status.RETRACTING;
      }
    } else {
      System.out.println("[ARM] Arm not initialized, cannot set length");
    }
  }

  public void reset() {
    // TODO: Move arm to safe angle for retracting
    m_status = status.RESSETING;

    System.out.println("[ARM] Resetting Initiated");
  }

  public boolean atExtensionSetpoint() {
    return MathUtil.isNear(m_desiredExtensionInches, m_currentExtInches, 1);
  }

  private boolean isInRange(double extensionInches) {
    boolean retval = true;
    if ((extensionInches < ExtConstants.kMinExtInches)
        || (extensionInches > ExtConstants.kMaxExtInches)) {
      retval = false;
    }
    return retval;
  }

  @Override
  public void periodic() {
    /** Check for value in Range * */
    m_desiredExtensionInches =
        MathUtil.clamp(m_desiredExtensionInches, Extension.kMinExtInches, Extension.kMaxExtInches);

    /** Update Current Arm Positions */
    m_currentExtInches = m_encoder.getPosition();

    /** Set Target Positions into the Controllers * */
    switch (m_status) {
      case RESSETING:
        if (m_extensionMotor_L.getOutputCurrent() < Extension.kZeroingCurrent) {
          // Resetting the Arm - Move backwards until Current raises for that motor
          m_extensionMotor_L.set(-0.5);
        } else {
          m_extensionMotor_L.set(0);
          m_encoder.setPosition(0);
          m_currentExtInches = 0;
          m_status = status.IDLE;
        }
        break;

      case EXTENDING:
        if (atExtensionSetpoint()) {
          m_extensionMotor_L.set(0.5);
        } else {
          m_extensionMotor_L.set(0);
          m_status = status.IDLE;
        }
        break;

      case RETRACTING:
        if (atExtensionSetpoint()) {
          m_extensionMotor_L.set(-0.5);
        } else {
          m_extensionMotor_L.set(0);
          m_status = status.IDLE;
        }
        break;

      case UNKNOWN:
      case IDLE:
    }
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Arm/Extension/DesiredInches", m_desiredExtensionInches);
    SmartDashboard.putNumber("Arm/Extension/CurrentInches", m_currentExtInches);
    SmartDashboard.putNumber("Arm/Extension/AMPs_L", m_extensionMotor_L.getOutputCurrent());
    SmartDashboard.putString("Arm/Extension/Status", m_status.getString());
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
