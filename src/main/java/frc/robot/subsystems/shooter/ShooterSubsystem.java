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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Presets.Preset;
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates.ShooterStatus;

public class ShooterSubsystem extends SubsystemBase implements ImplementDashboard {
  private CANSparkMax m_motor1;
  private CANSparkMax m_motor2;
  private SparkPIDController m_pidController1;
  private SparkPIDController m_pidController2;

  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;

  private double m_desiredRPM_Bottom = Preset.kShootSpeaker_0m.getBottomRPM();
  private double m_desiredRPM_Top = Preset.kShootSpeaker_0m.getTopRPM();
  private double m_currentRPM_Bottom = 0;
  private double m_currentRPM_Top = 0;

  private boolean m_enabled = false;
  private boolean m_isAtSpeed = false;
  private ShooterStatus m_status = ShooterStatus.IDLE;

  private double m_kP = 0.00015;
  private double m_kI = 0;
  private double m_kD = 0.00038;
  private double m_kIz = 0;
  private double m_kFF = 0.00017;
  private double m_kMaxOutput = 1;
  private double m_kMinOutput = -1;

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
    m_motor1 = new CANSparkMax(ShooterConstants.kCANMotor1, MotorType.kBrushless);
    m_motor1.restoreFactoryDefaults();
    m_motor1.setIdleMode(IdleMode.kCoast);
    m_motor1.setInverted(true);

    m_motor2 = new CANSparkMax(ShooterConstants.kCANMotor2, MotorType.kBrushless);
    m_motor2.restoreFactoryDefaults();
    m_motor2.setIdleMode(IdleMode.kCoast);
    m_motor2.setInverted(true);

    m_pidController1 = m_motor1.getPIDController();
    m_pidController2 = m_motor2.getPIDController();

    m_encoder1 = m_motor1.getEncoder();
    m_encoder2 = m_motor2.getEncoder();

    // set PID coefficients
    m_pidController1.setP(m_kP);
    m_pidController1.setI(m_kI);
    m_pidController1.setD(m_kD);
    m_pidController1.setIZone(m_kIz);
    m_pidController1.setFF(m_kFF);
    m_pidController1.setOutputRange(m_kMinOutput, m_kMaxOutput);

    m_pidController2.setP(m_kP);
    m_pidController2.setI(m_kI);
    m_pidController2.setD(m_kD);
    m_pidController2.setIZone(m_kIz);
    m_pidController2.setFF(m_kFF);
    m_pidController2.setOutputRange(m_kMinOutput, m_kMaxOutput);
  }

  public void setMotors(double speed) {
    m_pidController1.setReference(speed, CANSparkMax.ControlType.kVelocity);
    m_pidController2.setReference(speed, CANSparkMax.ControlType.kVelocity);
    // m_motor1.set(speed);
    // m_motor2.set(speed);
  }

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
        // m_motor1.set(0.9);
        // m_motor2.set(0.9);

        m_pidController1.setReference(m_desiredRPM_Bottom, CANSparkMax.ControlType.kVelocity);
        m_pidController2.setReference(m_desiredRPM_Top, CANSparkMax.ControlType.kVelocity);
      } else {
        m_pidController1.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        m_pidController2.setReference(0, CANSparkMax.ControlType.kDutyCycle);
      }
      m_currentRPM_Bottom = m_encoder1.getVelocity();
      m_currentRPM_Top = m_encoder2.getVelocity();

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
  public void initDashboard() {}

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

    double p = SmartDashboard.getNumber("Shooter/P Gain", m_kP);
    double d = SmartDashboard.getNumber("Shooter/D Gain", m_kD);
    double ff = SmartDashboard.getNumber("Shooter/Feed Forward", m_kFF);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != m_kP)) {
      m_pidController1.setP(p);
      m_pidController2.setP(p);
      m_kP = p;
    }
    if ((d != m_kD)) {
      m_pidController1.setD(d);
      m_pidController2.setD(d);
      m_kD = d;
    }
    if ((ff != m_kFF)) {
      m_pidController1.setFF(ff);
      m_pidController2.setFF(ff);
      m_kFF = ff;
    }

    SmartDashboard.putString("Shooter/Status", m_status.getString());
    SmartDashboard.putBoolean("Shooter/ENABLED", m_enabled);
    m_enabled = SmartDashboard.getBoolean("Shooter/ENABLED", m_enabled);
    SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());

    SmartDashboard.putNumber("Shooter/DesiredRPM_Bottom", m_desiredRPM_Bottom);
    SmartDashboard.putNumber("Shooter/DesiredRPM_Top", m_desiredRPM_Top);

    SmartDashboard.putNumber("Shooter/CurrentRPM_Bottom", m_currentRPM_Bottom);
    SmartDashboard.putNumber("Shooter/CurrentRPM_Top", m_currentRPM_Top);

    SmartDashboard.putNumber("Shooter/P Gain", m_kP);
    SmartDashboard.putNumber("Shooter/D Gain", m_kD);
    SmartDashboard.putNumber("Shooter/Feed Forward", m_kFF);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
