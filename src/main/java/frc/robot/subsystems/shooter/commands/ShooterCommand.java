// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemsConstants.Preset;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  private ShooterSubsystem m_shooter;

  private IntakeSubsystem m_intake;

  private double m_speedTop;
  private double m_speedBottom;

  public ShooterCommand(IntakeSubsystem intake, ShooterSubsystem shooter, Preset preset) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
    m_shooter = shooter;
    m_intake = intake;
    m_speedTop = preset.getTopRPM();
    m_speedBottom = preset.getBottomRPM();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setRPM(m_speedBottom, m_speedTop);
    m_shooter.start();
    LEDSubsystem.setMode(LedMode.SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotors(0);
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_intake.isFull();
  }
}
