// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class RetractClimberCommand extends Command {
  private final ClimberSubsystem m_climber;
  public RetractClimberCommand(ClimberSubsystem climber) {
    m_climber = climber;
    setName("Retract Climber");
    addRequirements(climber);
    withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setMotors(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
