// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wesbot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wesbot.WesBotSubsystem;

public class WesShootCommand extends Command {
 
private WesBotSubsystem m_shooter;
private double m_output;

  public WesShootCommand(WesBotSubsystem shooter, double output) {
    m_shooter = shooter;
    m_output = output;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setMotors(m_output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotors(0);
  }

}
