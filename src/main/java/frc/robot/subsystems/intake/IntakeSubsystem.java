// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

<<<<<<< HEAD
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
=======
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class IntakeSubsystem extends SubsystemBase implements ImplementDashboard {
  // private CANSparkMax m_motor1 = new CANSparkMax(IntakeConstants.kCANMotor1,
  // MotorType.kBrushless);
<<<<<<< HEAD
  private CANSparkMax m_motor1;
  private CANSparkMax m_motor2;
=======
  private VictorSPX m_motor1 = new VictorSPX(IntakeConstants.kCANMotor1);
  private VictorSPX m_motor2 = new VictorSPX(IntakeConstants.kCANMotor2);
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
  private DigitalInput m_infrared1 = new DigitalInput(IntakeConstants.kIRSensor1);
  private DigitalInput m_infrared2 = new DigitalInput(IntakeConstants.kIRSensor2);

  private double m_speed = IntakeConstants.kSpeed_Intake;
  private boolean m_enabled = false;

  /** Creates a new IntakeSubsystem. */
<<<<<<< HEAD
  public IntakeSubsystem() {
    m_motor1 = new CANSparkMax(IntakeConstants.kCANMotor1, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(IntakeConstants.kCANMotor2, MotorType.kBrushless);

    m_motor1.setInverted(true);
    m_motor2.setInverted(true);
  }

  public void setSpeed(double speed) {
    m_motor1.set(speed);
    m_motor2.set(speed);
  }

  public void start() {
    m_motor1.set(m_speed);
    m_motor2.set(m_speed);
    m_enabled = true;
  }

  public void stop() {
    m_motor1.set(0);
    m_motor2.set(0);
    m_enabled = false;
  }

=======
  public IntakeSubsystem() {}

  public void setMotors(double speed) {
    m_motor1.set(VictorSPXControlMode.PercentOutput, speed);
    m_motor2.set(VictorSPXControlMode.PercentOutput, speed);
  }

>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
  public boolean isFull() {
    // 0 means something is there, 1 means nothing is there
    return m_infrared1.get();
  }

  public boolean limitReached() {
    return !m_infrared2.get();
  }
<<<<<<< HEAD

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/ENABLED", m_enabled);
    SmartDashboard.putNumber("Intake/SetSpeed", m_speed);

    SmartDashboard.putBoolean("Intake Loaded", !m_infrared1.get());
    SmartDashboard.putBoolean("Intake/BottomSensor", !m_infrared1.get());
    SmartDashboard.putBoolean("Intake/ExitSensor", !m_infrared1.get());
  }
=======

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Limit Switch", limitReached());
  }

  @Override
  public void initDashboard() {}
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Intake/ENABLED", m_enabled);
    SmartDashboard.putNumber("Intake/SetSpeed", m_speed);

    SmartDashboard.putBoolean("Intake Loaded", !m_infrared1.get());
    SmartDashboard.putBoolean("Intake/BottomSensor", !m_infrared1.get());
    SmartDashboard.putBoolean("Intake/ExitSensor", !m_infrared1.get());
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
