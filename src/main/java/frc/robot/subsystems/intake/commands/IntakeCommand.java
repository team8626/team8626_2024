// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {
  private IntakeSubsystem m_intake;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intake) {
    addRequirements(intake);
    m_intake = intake;

    andThen(
        new IntakeAdjustmentCommand(intake)
            .onlyIf(() -> !m_intake.isFull() && m_intake.limitReached()));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    // TODO: m_intake.setMotors(1);
=======
    m_intake.setMotors(1);
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean yes = true;
    if (yes && m_intake.isFull()) {
<<<<<<< HEAD
      // TODO: m_intake.setMotors(0.4);
=======
      m_intake.setMotors(0.4);
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
      yes = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<< HEAD
    // TODO: m_intake.setMotors(0);
=======
    m_intake.setMotors(0);
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.limitReached();
  }
}
