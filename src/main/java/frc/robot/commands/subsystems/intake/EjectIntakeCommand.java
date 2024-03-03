// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class EjectIntakeCommand extends Command {
  IntakeSubsystem m_intake;
  Timer m_timer = new Timer();

  public EjectIntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_intake.start(IntakeConstants.kSpeed_Discard);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(1.5);
  }
}
