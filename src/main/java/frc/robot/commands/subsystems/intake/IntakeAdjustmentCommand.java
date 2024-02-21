// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates.IntakeStatus;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeAdjustmentCommand extends Command {
  private IntakeSubsystem m_intake;
  /** Creates a new IntakeAdjustmentCommand. */
  public IntakeAdjustmentCommand(IntakeSubsystem intake) {
    addRequirements(intake);
    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.start(IntakeConstants.kSpeed_Adjust);
    m_intake.setStatus(IntakeStatus.ADJUSTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_intake.setStatus(IntakeStatus.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_intake.isFull() && !m_intake.limitReached());
  }
}
