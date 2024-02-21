// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets.Preset;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SimpleShooterCommand extends Command {
  /** Creates a new SimpleShooterCommand. */
  private ShooterSubsystem m_shooter;

  private double m_speedTop;
  private double m_speedBottom;

  public SimpleShooterCommand(Preset preset, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    m_shooter = shooter;
    m_speedBottom = preset.getBottomRPM();
    m_speedTop = preset.getTopRPM();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setRPM(m_speedBottom, m_speedTop);
    m_shooter.start();
    // LEDSubsystem.setMode(LedMode.SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LEDSubsystem.setMode(LedMode.SHOOTING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
