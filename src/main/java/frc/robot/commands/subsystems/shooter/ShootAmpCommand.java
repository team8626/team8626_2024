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
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootAmpCommand extends Command {
  /** Creates a new SimpleShooterCommand. */
  private ShooterSubsystem m_shooter;

  private IntakeSubsystem m_intake;
  private Timer m_timer;

  public ShootAmpCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
    m_shooter = shooter;
    m_intake = intake;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.startDutyCycle(0.4);
    m_intake.start(IntakeConstants.kSpeed_Shoot);

    m_timer.reset();
    m_timer.start();
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
    m_intake.stop();
    m_shooter.stop();
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retval = false;

    if (m_timer.hasElapsed(1)) {
      retval = true;
    }
    return retval;
  }
}
