// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.preset.Presets.Preset;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates.ShooterStatus;

public class ShooterSubsystem extends SubsystemBase implements ImplementDashboard {
  private CANSparkMax m_motor_Bottom;
  private CANSparkMax m_motor_Top;
  private SparkPIDController m_pidController_Bottom;
  private SparkPIDController m_pidController_Top;

  private RelativeEncoder m_encoder_Bottom;
  private RelativeEncoder m_encoder_Top;

  private double m_desiredRPM_Bottom = Preset.kShootSpeaker_0m.getBottomRPM();
  private double m_desiredRPM_Top = Preset.kShootSpeaker_0m.getTopRPM();
  private double m_currentRPM_Bottom = 0;
  private double m_currentRPM_Top = 0;

  private boolean m_enabled = false;
  private boolean m_isAtSpeed = false;
  private ShooterStatus m_status = ShooterStatus.IDLE;

  private double m_kP_top;
  private double m_kI_top;
  private double m_kD_top;
  private double m_kIz_top;
  private double m_kFF_top;
  private double m_kMaxOutput_top;
  private double m_kMinOutput_top;

  private double m_kP_bottom;
  private double m_kI_bottom;
  private double m_kD_bottom;
  private double m_kIz_bottom;
  private double m_kFF_bottom;
  private double m_kMaxOutput_bottom;
  private double m_kMinOutput_bottom;

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {

    m_kP_top = ShooterConstants.kP_top;
    m_kI_top = ShooterConstants.kI_top;
    m_kD_top = ShooterConstants.kD_top;
    m_kIz_top = ShooterConstants.kIz_top;
    m_kFF_top = ShooterConstants.kFF_top;
    m_kMaxOutput_top = ShooterConstants.kMaxOutput_top;
    m_kMinOutput_top = ShooterConstants.kMinOutput_top;

    m_kP_bottom = ShooterConstants.kP_bottom;
    m_kI_bottom = ShooterConstants.kI_bottom;
    m_kD_bottom = ShooterConstants.kD_bottom;
    m_kIz_bottom = ShooterConstants.kIz_bottom;
    m_kFF_bottom = ShooterConstants.kFF_bottom;
    m_kMaxOutput_bottom = ShooterConstants.kMaxOutput_bottom;
    m_kMinOutput_bottom = ShooterConstants.kMinOutput_bottom;

    m_motor_Bottom = new CANSparkMax(ShooterConstants.kCANMotor1, MotorType.kBrushless);
    m_motor_Bottom.restoreFactoryDefaults();
    m_motor_Bottom.setIdleMode(IdleMode.kCoast);
    m_motor_Bottom.setInverted(true);

    m_motor_Top = new CANSparkMax(ShooterConstants.kCANMotor2, MotorType.kBrushless);
    m_motor_Top.restoreFactoryDefaults();
    m_motor_Top.setIdleMode(IdleMode.kCoast);
    m_motor_Top.setInverted(true);

    m_pidController_Bottom = m_motor_Bottom.getPIDController();
    m_pidController_Top = m_motor_Top.getPIDController();

    m_encoder_Bottom = m_motor_Bottom.getEncoder();
    m_encoder_Top = m_motor_Top.getEncoder();

    // set PID coefficients
    m_pidController_Bottom.setP(m_kP_bottom);
    m_pidController_Bottom.setI(m_kI_bottom);
    m_pidController_Bottom.setD(m_kD_bottom);
    m_pidController_Bottom.setIZone(m_kIz_bottom);
    m_pidController_Bottom.setFF(m_kFF_bottom);
    m_pidController_Bottom.setOutputRange(m_kMinOutput_bottom, m_kMaxOutput_bottom);

    m_pidController_Top.setP(m_kP_top);
    m_pidController_Top.setI(m_kI_top);
    m_pidController_Top.setD(m_kD_top);
    m_pidController_Top.setIZone(m_kIz_top);
    m_pidController_Top.setFF(m_kFF_top);
    m_pidController_Top.setOutputRange(m_kMinOutput_top, m_kMaxOutput_top);

    // SmartDashboard.putNumber("Shooter/PGainBottom", 0);
    // SmartDashboard.putNumber("Shooter/IGainBottom", 0);
    // SmartDashboard.putNumber("Shooter/DGainBottom", 0);
    // SmartDashboard.putNumber("Shooter/FFBottom", 0);
  }

  // TODO: Remove
  // public void setBottomPIDs(double p, double i, double d, double iZone) {
  // m_pidController_Bottom.setP(p);
  // m_pidController_Bottom.setI(i);
  // m_pidController_Bottom.setD(d);
  // m_pidController_Bottom.setIZone(iZone);
  // }

  public void setRPM(double speed_Bottom, double speed_Top) {
    m_desiredRPM_Bottom = speed_Bottom;
    m_desiredRPM_Top = speed_Top;

    // Force Dashboard Update
    initDashboard();
  }

  public void setRPM(double speed) {
    m_desiredRPM_Bottom = speed;
    m_desiredRPM_Top = speed;

    // Force Dashboard Update
    initDashboard();
  }

  public void start(Preset newPreset) {
    this.setRPM(newPreset.getBottomRPM(), newPreset.getTopRPM());
    this.start();
  }

  public void start() {
    m_enabled = true;
  }

  public void stop() {
    m_enabled = false;
  }

  public boolean isAtSpeed() {
    return m_isAtSpeed;
  }

  private void checkRPM() {
    if (m_enabled) {
      if (MathUtil.isNear(m_desiredRPM_Bottom, m_currentRPM_Bottom, ShooterConstants.kRPMTolerance)
          && MathUtil.isNear(m_desiredRPM_Top, m_currentRPM_Top, ShooterConstants.kRPMTolerance)) {
        m_isAtSpeed = true;
        m_status = ShooterStatus.ATSPEED;
      } else {
        m_isAtSpeed = false;
        m_status = ShooterStatus.RAMPUP;
      }
    } else {
      m_isAtSpeed = false;
      m_status = ShooterStatus.IDLE;
    }
  }

  @Override
  public void periodic() {
    // Clamp Values
    m_desiredRPM_Bottom = MathUtil.clamp(m_desiredRPM_Bottom, 0, ShooterConstants.kMaxRPM);
    m_desiredRPM_Top = MathUtil.clamp(m_desiredRPM_Top, 0, ShooterConstants.kMaxRPM);

    if (Robot.isReal()) {
      if (m_enabled) {
        m_pidController_Bottom.setReference(m_desiredRPM_Bottom, CANSparkMax.ControlType.kVelocity);
        m_pidController_Top.setReference(m_desiredRPM_Top, CANSparkMax.ControlType.kVelocity);
      } else {
        m_pidController_Bottom.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        m_pidController_Top.setReference(0, CANSparkMax.ControlType.kDutyCycle);
      }
      m_currentRPM_Bottom = m_encoder_Bottom.getVelocity();
      m_currentRPM_Top = m_encoder_Top.getVelocity();

    } else if (Robot.isSimulation()) {
      if (m_enabled) {
        if (m_desiredRPM_Bottom < m_currentRPM_Bottom) {
          m_currentRPM_Bottom = Math.max(m_desiredRPM_Bottom, m_currentRPM_Bottom - 100);
        } else if (m_desiredRPM_Bottom > m_currentRPM_Bottom) {
          m_currentRPM_Bottom = Math.min(m_desiredRPM_Bottom, m_currentRPM_Bottom + 100);
        }

        if (m_desiredRPM_Top < m_currentRPM_Top) {
          m_currentRPM_Top = Math.max(m_desiredRPM_Top, m_currentRPM_Top - 100);
        } else if (m_desiredRPM_Top > m_currentRPM_Top) {
          m_currentRPM_Top = Math.min(m_desiredRPM_Top, m_currentRPM_Top + 100);
        }
      } else {
        m_currentRPM_Bottom = 0;
        m_currentRPM_Top = 0;
      }
    }
    checkRPM();
  }

  private void setStatus(ShooterStatus newStatus) {
    m_status = newStatus;
    System.out.printf("[SHOOTER] New Status: %s\n", m_status.getString());
  }

  @Override
  public void initDashboard() {
    SmartDashboard.putNumber("Shooter/P Gain Top", m_kP_top);
    SmartDashboard.putNumber("Shooter/I Gain Top ", m_kI_top);
    SmartDashboard.putNumber("Shooter/D Gain Top", m_kD_top);
    SmartDashboard.putNumber("Shooter/Feed Forward Top", m_kFF_top);
    SmartDashboard.putNumber("Shooter/P Gain Bottom", m_kP_bottom);
    SmartDashboard.putNumber("Shooter/I Gain Bottom ", m_kI_bottom);
    SmartDashboard.putNumber("Shooter/D Gain Bottom", m_kD_bottom);
    SmartDashboard.putNumber("Shooter/Feed Forward Bottom", m_kFF_bottom);
  }

  @Override
  public void updateDashboard() {
    double newRPM_Bottom =
        SmartDashboard.getNumber("Shooter/DesiredRPM_Bottom", m_desiredRPM_Bottom);
    double newRPM_Top = SmartDashboard.getNumber("Shooter/DesiredRPM_Top", m_desiredRPM_Top);

    if (newRPM_Bottom != m_desiredRPM_Bottom) {
      m_desiredRPM_Bottom = newRPM_Bottom;
    }
    if (newRPM_Top != m_desiredRPM_Top) {
      m_desiredRPM_Top = newRPM_Top;
    }

    double p_top = SmartDashboard.getNumber("Shooter/P Gain Top", m_kP_top);
    double i_top = SmartDashboard.getNumber("Shooter/I Gain Top", m_kI_top);
    double d_top = SmartDashboard.getNumber("Shooter/D Gain Top", m_kD_top);
    double ff_top = SmartDashboard.getNumber("Shooter/Feed Forward Top", m_kFF_top);
    double p_bottom = SmartDashboard.getNumber("Shooter/P Gain Bottom", m_kP_bottom);
    double i_bottom = SmartDashboard.getNumber("Shooter/I Gain Bottom", m_kI_bottom);
    double d_bottom = SmartDashboard.getNumber("Shooter/D Gain Bottom", m_kD_bottom);
    double ff_bottom = SmartDashboard.getNumber("Shooter/Feed Forward Bottom", m_kFF_bottom);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p_top != m_kP_top)) {
      System.out.printf("[SHOOTER] Updating Top P to %f, was %f\n", m_kP_top, p_top);

      m_pidController_Top.setP(p_top);
      m_kP_top = p_top;
    }
    if ((i_top != m_kI_top)) {
      System.out.printf("[SHOOTER] Updating Top I to %f, was %f\n", m_kI_top, i_top);
      m_pidController_Top.setI(i_top);
      m_kI_top = i_top;
    }
    if ((d_top != m_kD_top)) {
      m_pidController_Top.setD(d_top);
      m_kD_top = d_top;
    }
    if ((ff_top != m_kFF_top)) {
      m_pidController_Top.setFF(ff_top);
      m_kFF_top = ff_top;
    }

    if ((p_bottom != m_kP_bottom)) {
      m_pidController_Bottom.setP(p_bottom);
      m_kP_bottom = p_bottom;
    }
    if ((i_bottom != m_kI_bottom)) {
      m_pidController_Bottom.setI(i_bottom);
      m_kI_bottom = i_bottom;
    }

    if ((d_bottom != m_kD_bottom)) {
      m_pidController_Bottom.setD(d_bottom);
      m_kD_bottom = d_bottom;
    }

    if ((ff_bottom != m_kFF_bottom)) {
      m_pidController_Bottom.setFF(ff_bottom);
      m_kFF_bottom = ff_bottom;
    }

    SmartDashboard.putString("Shooter/Status", m_status.getString());
    SmartDashboard.putBoolean("Shooter/ENABLED", m_enabled);
    SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());

    SmartDashboard.putNumber("Shooter/DesiredRPM_Bottom", m_desiredRPM_Bottom);
    SmartDashboard.putNumber("Shooter/DesiredRPM_Top", m_desiredRPM_Top);

    SmartDashboard.putNumber("Shooter/CurrentRPM_Bottom", m_currentRPM_Bottom);
    SmartDashboard.putNumber("Shooter/CurrentRPM_Top", m_currentRPM_Top);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }

  public Command startCmd() {
    return startEnd(() -> this.m_enabled = true, () -> this.m_enabled = false)
        .withName("[SHOOTER] StartCmd " + m_desiredRPM_Top + "/" + m_desiredRPM_Bottom);
  }
}
