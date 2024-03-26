// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.extension;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.preset.Presets;

public class ArmExtensionSubsystem extends SubsystemBase implements ImplementDashboard {

  /** Create Motors * */
  private final CANSparkMax m_extensionMotor_L;

  /** Create Sensors * */
  // private final RelativeEncoder m_extensionEncoder;

  private final RelativeEncoder m_theOtherExtensionEncoder;

  private final SparkPIDController m_theOtherPIDController;

  /** Class States * */
  private double m_desiredExtensionInches = 0;

  private double m_currentExtInches = 0;

  private boolean m_armIsResetting = false;
  private boolean m_armZeroed = false;

  private double m_kExtensionP = ExtConstants.kExtP;
  private double m_kExtensionI = ExtConstants.kExtI;
  private double m_kExtensionD = ExtConstants.kExtD;
  private double m_kExtensionFF = ExtConstants.kExtFF;
  private double m_kExtensionMaxOutput = ExtConstants.kExtMaxOutput;
  private double m_kExtensionMinOutput = ExtConstants.kExtMinOutput;

  /** Creates a new ArmExtensionSubsystem. */
  public ArmExtensionSubsystem() {
    // Motors
    m_extensionMotor_L = new CANSparkMax(ExtConstants.extensionCANID_L, MotorType.kBrushless);

    // Reset & Initialize Controllers
    m_extensionMotor_L.restoreFactoryDefaults();

    // Setup encoders and PID controllers for SPARKMAX.
    // m_extensionEncoder = m_extensionMotor_L.getAlternateEncoder(Extension.kEncType,
    // Extension.kCPR);
    m_theOtherExtensionEncoder = m_extensionMotor_L.getEncoder();

    m_theOtherPIDController = m_extensionMotor_L.getPIDController();
    m_theOtherPIDController.setFeedbackDevice(m_theOtherExtensionEncoder);
    m_theOtherPIDController.setP(m_kExtensionP);
    m_theOtherPIDController.setI(m_kExtensionI);
    m_theOtherPIDController.setD(m_kExtensionD);
    m_theOtherPIDController.setFF(m_kExtensionFF);
    m_theOtherPIDController.setOutputRange(m_kExtensionMinOutput, m_kExtensionMaxOutput);

    // m_extensionPIDController = m_extensionMotor_L.getPIDController();
    // m_extensionPIDController.setFeedbackDevice(m_theOtherExtensionEncoder);

    // Extension Soft Limits
    m_extensionMotor_L.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_extensionMotor_L.enableSoftLimit(SoftLimitDirection.kForward, false);

    // Apply position and velocity conversion factors for the turning encoder.
    // We want these in degrees and degrees per second
    m_extensionMotor_L.setInverted(false);
    m_theOtherExtensionEncoder.setPositionConversionFactor(
        ExtConstants.kExtensionEncoderPositionFactorDeg);
    m_theOtherExtensionEncoder.setVelocityConversionFactor(
        ExtConstants.kExtensionEncoderVelocityFactorDeg);

    // Idle Modes and max current
    m_extensionMotor_L.setIdleMode(IdleMode.kBrake);
    m_extensionMotor_L.setSmartCurrentLimit(ExtConstants.kCurrentLimit);

    // Write configuration to Controllers
    m_extensionMotor_L.burnFlash();

    // FORCE DASHBOARD UPDATE (Get values)
    updateDashboard();

    if (RobotBase.isReal()) {
      this.reset();
    } else if (RobotBase.isSimulation()) {
      m_desiredExtensionInches = Presets.kStart.getExtInches();
      m_currentExtInches = m_desiredExtensionInches;
    }
  }

  /**
   * Set Arm Length to a specific extension in Inches.
   *
   * @param newLengthInches
   */
  public void setLengthInches(double newLengthInches) {
    m_desiredExtensionInches = newLengthInches; // Value Clamped in periodic()
    // System.out.printf("[ARM] New Length: %f\n", m_desiredExtensionInches);
  }

  private void extend(double newSpeed) {
    if (Robot.isReal()) {
      if ((m_currentExtInches < ExtConstants.kMaxExtInches)
          && (m_currentExtInches > ExtConstants.kMinExtInches)) {
        m_extensionMotor_L.set(0.5);
      }
    } else { // Simulation
      if (newSpeed > 0) {
        m_desiredExtensionInches += 0.5;
      } else if (newSpeed < 0) {
        m_desiredExtensionInches -= 0.5;
      }
    }
  }

  public void setMotorsMode(IdleMode mode) {
    m_extensionMotor_L.setIdleMode(mode);
  }

  public void reset() {
    // TODO: Move arm to safe angle for retracting
    m_armZeroed = false;
    m_armIsResetting = true;

    System.out.println("[ARM] Resetting Initiated");
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(
        m_desiredExtensionInches, m_currentExtInches, ExtConstants.kAtInchesTolerance);
  }

  private boolean isInRange(double extensionInches) {
    boolean retval = true;
    if ((extensionInches < ExtConstants.kMinExtInches)
        || (extensionInches > ExtConstants.kMaxExtInches)) {
      retval = false;
    }
    return retval;
  }

  public double getExtInches() {
    return m_currentExtInches;
  }

  @Override
  public void periodic() {

    /** Check for value in Range * */
    m_desiredExtensionInches =
        MathUtil.clamp(
            m_desiredExtensionInches, ExtConstants.kMinExtInches, ExtConstants.kMaxExtInches);

    /** Update Current Arm Positions * */
    if (RobotBase.isReal()) {
      m_currentExtInches = m_theOtherExtensionEncoder.getPosition();
    } else {
      updateSimValues();
    }

    /** Set Target Positions into the Controllers * */
    if (!m_armIsResetting) {
      if (m_armZeroed) {
        m_theOtherPIDController.setReference(m_desiredExtensionInches, ControlType.kPosition);
      } else {
        // Not Zeroed and Not Resetting
        // Something when wrong, Force Reset!
        this.reset();
      }
    } else {
      /** Use Motor output current to detect low limit * */
      if (m_extensionMotor_L.getOutputCurrent() < ExtConstants.kZeroingCurrent) {
        // Resetting the Arm - Move backwards until Current raises for that motor
        m_extensionMotor_L.set(-0.5);
      }
      /** Reached current threshold, we found the zero!* */
      else {
        // Reset done!
        System.out.println("[ARM] Resetting DONE");

        m_theOtherExtensionEncoder.setPosition(0);
        m_extensionMotor_L.set(0);

        m_currentExtInches = 0;
        m_desiredExtensionInches = 0;
        m_armZeroed = true;
        m_armIsResetting = false;
      }
      // }
      /** Running simulation, Reset "done" * */
      if (Robot.isSimulation()) {
        m_currentExtInches = 0;
        m_desiredExtensionInches = 0;
        m_armZeroed = true;
        m_armIsResetting = false;
      }
    }
  }

  @Override
  public void initDashboard() {
    SmartDashboard.getNumber("Arm/Extension/P Gain", m_kExtensionP);
    SmartDashboard.getNumber("Arm/Extension/D Gain", m_kExtensionD);
    SmartDashboard.getNumber("Arm/Extension/Feed Forward", m_kExtensionFF);
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Arm/Extension/DesiredInches", m_desiredExtensionInches);
    SmartDashboard.putNumber("Arm/Extension/CurrentInches", m_currentExtInches);
    SmartDashboard.putNumber("Arm/Extension/AMPs_L", m_extensionMotor_L.getOutputCurrent());
    SmartDashboard.putBoolean("Arm/Extension/atSetPoint", atSetpoint());

    SmartDashboard.putBoolean("Arm/Extension/Resetting", m_armIsResetting);
    SmartDashboard.putBoolean("Arm/Extension/Zeroed", m_armZeroed);

    double extP = SmartDashboard.getNumber("Arm/Extension/P Gain", ExtConstants.kExtP);
    double extD = SmartDashboard.getNumber("Arm/Extension/D Gain", ExtConstants.kExtD);
    double extFF = SmartDashboard.getNumber("Arm/Extension/Feed Forward", ExtConstants.kExtFF);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((extP != m_kExtensionP)) {
      m_theOtherPIDController.setP(extP);
      // m_extensionPIDController.setP(extP);

      m_kExtensionP = extP;
    }
    if ((extD != m_kExtensionD)) {
      m_theOtherPIDController.setP(extD);
      // m_extensionPIDController.setD(extD);
      m_kExtensionD = extD;
    }
    if ((extFF != m_kExtensionFF)) {
      m_theOtherPIDController.setP(extFF);
      // m_extensionPIDController.setFF(extFF);
      m_kExtensionFF = extFF;
    }

    SmartDashboard.putNumber("Arm/Extension/P Gain", m_kExtensionP);
    SmartDashboard.putNumber("Arm/Extension/D Gain", m_kExtensionD);
    SmartDashboard.putNumber("Arm/Extension/Feed Forward", m_kExtensionFF);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }

  private void updateSimValues() {
    if (m_currentExtInches < m_desiredExtensionInches) {
      m_currentExtInches += 0.1;
    } else if (m_currentExtInches > m_desiredExtensionInches) {
      m_currentExtInches -= 0.1;
    }

    // Clamp values for stable simulation
    m_currentExtInches =
        MathUtil.clamp(m_currentExtInches, ExtConstants.kMinExtInches, ExtConstants.kMaxExtInches);
  }
}
