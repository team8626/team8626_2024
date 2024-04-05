// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates.IntakeStatus;

public class IntakeSubsystem extends SubsystemBase implements ImplementDashboard {
  // private CANSparkMax m_motor1 = new CANSparkMax(IntakeConstants.kCANMotor1,
  // MotorType.kBrushless);
  private CANSparkMax m_motor1; // Bottom Motor
  private CANSparkMax m_motor2; // Top Motor
  private DigitalInput m_infrared1 = new DigitalInput(IntakeConstants.kIRSensor1);
  private DigitalInput m_infrared2 = new DigitalInput(IntakeConstants.kIRSensor2);

  private double m_speed = IntakeConstants.kSpeed_Intake;
  private boolean m_enabled = false;
  private IntakeStatus m_status = IntakeStatus.IDLE;

  private boolean m_bottomSensor = false;
  private boolean m_exitSensor = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_motor1 = new CANSparkMax(IntakeConstants.kCANMotor1, MotorType.kBrushless);
    m_motor2 = new CANSparkMax(IntakeConstants.kCANMotor2, MotorType.kBrushless);

    m_motor1.setInverted(false);
    m_motor2.setInverted(false);
  }

  public void setSpeed(double speed) {
    m_speed = speed;
  }

  public void start() {
    m_motor1.set(m_speed);
    m_motor2.set(m_speed);
    m_enabled = true;
  }

  public void start(double speed) {
    setSpeed(speed);
    start();
  }

  public void stop() {
    m_motor1.set(0);
    m_motor2.set(0);
    m_enabled = false;
  }

  /**
   * Checks if a NOTE is loaded in the Intake
   *
   * @return true if NOTE is loaded
   */
  public boolean isFull() {
    // 0 means something is there, 1 means nothing is there
    return m_bottomSensor;
  }

  /**
   * Checks if a NOTE is present in the Intake
   *
   * @return true ifno NOTE is loaded
   */
  public boolean isEmpty() {
    return !(m_bottomSensor || m_exitSensor);
  }

  /**
   * Checks if a NOTE was "overshot" in the Intake
   *
   * @return true if NOTE is loaded
   */
  public boolean limitReached() {
    if (Robot.isReal()) {}
    return m_exitSensor;
  }

  @Override
  public void periodic() {
    if (Robot.isReal()) {
      m_bottomSensor = !m_infrared1.get();
      m_exitSensor = !m_infrared2.get();
    } else if (Robot.isSimulation()) {
      m_bottomSensor = SmartDashboard.getBoolean("Intake/BottomSensor", m_bottomSensor);
      m_exitSensor = SmartDashboard.getBoolean("Intake/ExitSensor", m_exitSensor);
    }
  }

  public void setStatus(IntakeStatus newStatus) {
    m_status = newStatus;
    System.out.printf("[INTAKE] New Status: %s\n", m_status.getString());
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Intake/ENABLED", m_enabled);
    SmartDashboard.putString("Intake/Status", m_status.getString());

    SmartDashboard.putNumber("Intake/SetSpeed", m_speed);

    SmartDashboard.putBoolean("Intake/IsEmpty", isEmpty());
    SmartDashboard.putBoolean("Intake/BottomSensor", m_bottomSensor);
    SmartDashboard.putBoolean("Intake/ExitSensor", m_exitSensor);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
