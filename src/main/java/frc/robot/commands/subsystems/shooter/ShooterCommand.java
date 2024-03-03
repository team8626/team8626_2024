// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.preset.Presets.Preset;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.Supplier;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  private ShooterSubsystem m_shooter;

  private IntakeSubsystem m_intake;

  private double m_speedTop;
  private double m_speedBottom;

  private Timer m_timer = new Timer();

  public ShooterCommand(IntakeSubsystem intake, ShooterSubsystem shooter, Supplier<Preset> preset) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
    m_shooter = shooter;
    m_intake = intake;

    m_speedTop = preset.get().getTopRPM();
    m_speedBottom = preset.get().getBottomRPM();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_shooter.setRPM(m_speedBottom, m_speedTop);
    m_shooter.start();
    m_intake.setSpeed(IntakeConstants.kSpeed_Shoot);
    LEDSubsystem.setMode(LedMode.SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.isAtSpeed()) {
      m_timer.start();
    }
    if (m_timer.hasElapsed(0.5)) {
      m_intake.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_intake.stop();
    m_timer.stop();
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retval = false;

    if (m_intake.isEmpty()) {
      m_timer.start();

      if (m_timer.hasElapsed(1)) {
        retval = true;
      }
    }

    return retval;
  }
}
