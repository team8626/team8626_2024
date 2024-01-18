// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitbotshooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kitbotshooter.KitBotShooterSubsystem;

public class AcitvateShooterCommand extends Command {

  private final KitBotShooterSubsystem m_shooter;
  private final double m_output;

  public AcitvateShooterCommand(KitBotShooterSubsystem shooter, double output) {
    m_shooter = shooter;
    m_output = output;

    setName("Acitvate Kitbot Shooter");
    addRequirements(m_shooter);
    withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setMotors(m_output);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
