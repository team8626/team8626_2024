// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

<<<<<<< HEAD
<<<<<<< HEAD
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
=======
import com.revrobotics.CANSparkBase.IdleMode;
>>>>>>> 5ff9b85 (Pre Debugging (other files))
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
<<<<<<< HEAD
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DigitalInput;
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
=======
=======
import edu.wpi.first.math.MathUtil;
>>>>>>> 5ff9b85 (Pre Debugging (other files))
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> 70353eb (Dashboard pre work)
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class ShooterSubsystem extends SubsystemBase implements ImplementDashboard {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 5ff9b85 (Pre Debugging (other files))
  private CANSparkMax m_motor1;
  private CANSparkMax m_motor2;
  private SparkPIDController m_pidController1;
  private SparkPIDController m_pidController2;
<<<<<<< HEAD

  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;

  private double m_desiredRPM_Bottom = ShooterConstants.kShootFromSpeakerRPM;
  private double m_desiredRPM_Top = ShooterConstants.kShootFromSpeakerRPM;
  private double m_currentRPM_Bottom = 0;
  private double m_currentRPM_Top = 0;
  private boolean m_enabled = false;

  private double m_kP = 0.0001;
  private double m_kI = 0;
  private double m_kD = 0;
  private double m_kIz = 0;
  private double m_kFF = 0.000016;
  private double m_kMaxOutput = 1;
  private double m_kMinOutput = -1;
=======
  private CANSparkMax m_motor1 = new CANSparkMax(ShooterConstants.kCANMotor1, MotorType.kBrushless);
  private CANSparkMax m_motor2 = new CANSparkMax(ShooterConstants.kCANMotor2, MotorType.kBrushless);
  private SparkPIDController m_pidController1 = m_motor1.getPIDController();
  private SparkPIDController m_pidController2 = m_motor2.getPIDController();
=======
>>>>>>> 5ff9b85 (Pre Debugging (other files))

  private RelativeEncoder m_encoder1;
  private RelativeEncoder m_encoder2;

  private double m_desiredRPM_Bottom = ShooterConstants.kShootFromSpeakerRPM;
  private double m_desiredRPM_Top = ShooterConstants.kShootFromSpeakerRPM;
  private double m_currentRPM_Bottom = 0;
<<<<<<< HEAD
>>>>>>> 70353eb (Dashboard pre work)

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
<<<<<<< HEAD
=======
  private double m_currentRPM_Top = 0;
  private boolean m_enabled = false;

  private double m_kP = 0.0001;
  private double m_kI = 0;
  private double m_kD = 0;
  private double m_kIz = 0;
  private double m_kFF = 0.000016;
  private double m_kMaxOutput = 1;
  private double m_kMinOutput = -1;

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {
>>>>>>> 5ff9b85 (Pre Debugging (other files))
    m_motor1 = new CANSparkMax(ShooterConstants.kCANMotor1, MotorType.kBrushless);
    m_motor1.restoreFactoryDefaults();
    m_motor1.setIdleMode(IdleMode.kCoast);
    m_motor1.setInverted(true);

    m_motor2 = new CANSparkMax(ShooterConstants.kCANMotor2, MotorType.kBrushless);
    m_motor2.restoreFactoryDefaults();
    m_motor1.setIdleMode(IdleMode.kCoast);
    m_motor2.setInverted(true);

    m_pidController1 = m_motor1.getPIDController();
    m_pidController2 = m_motor2.getPIDController();

    m_encoder1 = m_motor1.getEncoder();
    m_encoder2 = m_motor2.getEncoder();
<<<<<<< HEAD
=======
    double kP = 6e-5;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0.000015;
    double kMaxOutput = 1;
    double kMinOutput = -1;
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
=======
>>>>>>> 5ff9b85 (Pre Debugging (other files))

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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  public void setRPM(double speed_Bottom, double speed_Top) {
    m_desiredRPM_Bottom = speed_Bottom;
    m_desiredRPM_Top = speed_Top;

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
    boolean retval = true; // TODO: compute the setpoint
    // if( (Math.abs(m_encoder1.getVelocity() - m_RPMSetPoint1) > Shooter.kRPMTolerance)
    //   && (Math.abs(m_encoder2.getVelocity() - m_RPMSetPoint2) <= Shooter.kRPMTolerance) ){
    //   retval = false;
    // }
    return retval;
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
        // m_motor1.set(0);
        // m_motor2.set(0);
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
  }

  @Override
  public void initDashboard() {
    SmartDashboard.putBoolean("Shooter/ENABLED", m_enabled);

    SmartDashboard.putNumber("Shooter/DesiredRPM_Bottom", m_desiredRPM_Bottom);
    SmartDashboard.putNumber("Shooter/DesiredRPM_Top", m_desiredRPM_Top);

    SmartDashboard.putNumber("Shooter/CurrentRPM_Bottom", m_currentRPM_Bottom);
    SmartDashboard.putNumber("Shooter/CurrentRPM_Top", m_currentRPM_Top);

    SmartDashboard.putNumber("Shooter/P Gain", m_kP);
    SmartDashboard.putNumber("Shooter/D Gain", m_kD);
    SmartDashboard.putNumber("Shooter/Feed Forward", m_kFF);
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

    double p = SmartDashboard.getNumber("Shooter/P Gain", 0);
    double d = SmartDashboard.getNumber("Shooter/D Gain", 0);
    double ff = SmartDashboard.getNumber("Shooter/Feed Forward", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != m_kP)) {
      m_pidController1.setP(p);
      m_pidController2.setP(p);
      m_kP = p;
    }
    if ((d != m_kD)) {
      m_pidController1.setD(d);
      m_pidController2.setP(d);
      m_kD = d;
    }
    if ((ff != m_kFF)) {
      m_pidController1.setFF(ff);
      m_pidController2.setP(ff);
      m_kFF = ff;
    }

    SmartDashboard.putBoolean("Shooter/ENABLED", m_enabled);

    SmartDashboard.putNumber("Shooter/DesiredRPM_Bottom", m_desiredRPM_Bottom);
    SmartDashboard.putNumber("Shooter/DesiredRPM_Top", m_desiredRPM_Top);

    SmartDashboard.putNumber("Shooter/CurrentRPM_Bottom", m_currentRPM_Bottom);
    SmartDashboard.putNumber("Shooter/CurrentRPM_Top", m_currentRPM_Top);

    SmartDashboard.putNumber("Shooter/P Gain", m_kP);
    SmartDashboard.putNumber("Shooter/D Gain", m_kD);
    SmartDashboard.putNumber("Shooter/Feed Forward", m_kFF);
  }
=======
  public boolean isEmpty() {
    // 0 means something is there, 1 means nothing is there
    return m_infrared.get();
=======
  public void setRPM(double speed_top, double speed_bottom) {
    m_desiredRPM_Top = speed_top;
    m_desiredRPM_Bottom = speed_bottom;
=======
  public void setRPM(double speed_Bottom, double speed_Top) {
    m_desiredRPM_Bottom = speed_Bottom;
    m_desiredRPM_Top = speed_Top;
>>>>>>> 5ff9b85 (Pre Debugging (other files))

    // Force Dashboard Update
    initDashboard();
>>>>>>> 70353eb (Dashboard pre work)
  }

  public void start() {
    m_enabled = true;
  }

  public void stop() {
    m_enabled = false;
  }

  public boolean isAtSpeed() {
    boolean retval = true; // TODO: compute the setpoint
    // if( (Math.abs(m_encoder1.getVelocity() - m_RPMSetPoint1) > Shooter.kRPMTolerance)
    //   && (Math.abs(m_encoder2.getVelocity() - m_RPMSetPoint2) <= Shooter.kRPMTolerance) ){
    //   retval = false;
    // }
    return retval;
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
        // m_motor1.set(0);
        // m_motor2.set(0);
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
  }

  @Override
  public void initDashboard() {
    SmartDashboard.putBoolean("Shooter/ENABLED", m_enabled);

    SmartDashboard.putNumber("Shooter/DesiredRPM_Bottom", m_desiredRPM_Bottom);
    SmartDashboard.putNumber("Shooter/DesiredRPM_Top", m_desiredRPM_Top);

    SmartDashboard.putNumber("Shooter/CurrentRPM_Bottom", m_currentRPM_Bottom);
    SmartDashboard.putNumber("Shooter/CurrentRPM_Top", m_currentRPM_Top);

    SmartDashboard.putNumber("Shooter/P Gain", m_kP);
    SmartDashboard.putNumber("Shooter/D Gain", m_kD);
    SmartDashboard.putNumber("Shooter/Feed Forward", m_kFF);
  }

  @Override
<<<<<<< HEAD
  public void updateDashboard() {}
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
=======
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

    double p = SmartDashboard.getNumber("Shooter/P Gain", 0);
    double d = SmartDashboard.getNumber("Shooter/D Gain", 0);
    double ff = SmartDashboard.getNumber("Shooter/Feed Forward", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != m_kP)) {
      m_pidController1.setP(p);
      m_pidController2.setP(p);
      m_kP = p;
    }
    if ((d != m_kD)) {
      m_pidController1.setD(d);
      m_pidController2.setP(d);
      m_kD = d;
    }
    if ((ff != m_kFF)) {
      m_pidController1.setFF(ff);
      m_pidController2.setP(ff);
      m_kFF = ff;
    }

    SmartDashboard.putBoolean("Shooter/ENABLED", m_enabled);

    SmartDashboard.putNumber("Shooter/DesiredRPM_Bottom", m_desiredRPM_Bottom);
    SmartDashboard.putNumber("Shooter/DesiredRPM_Top", m_desiredRPM_Top);

    SmartDashboard.putNumber("Shooter/CurrentRPM_Bottom", m_currentRPM_Bottom);
    SmartDashboard.putNumber("Shooter/CurrentRPM_Top", m_currentRPM_Top);

    SmartDashboard.putNumber("Shooter/P Gain", m_kP);
    SmartDashboard.putNumber("Shooter/D Gain", m_kD);
    SmartDashboard.putNumber("Shooter/Feed Forward", m_kFF);
  }
>>>>>>> 70353eb (Dashboard pre work)

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
