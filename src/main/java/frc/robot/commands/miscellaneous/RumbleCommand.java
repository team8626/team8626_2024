// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.miscellaneous;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class RumbleCommand extends Command {

  GenericHID m_controller;
  Runnable m_execute;
  BooleanSupplier m_isFinished;
  private Timer m_timer = new Timer();
  private double m_intensity;
  private RumbleType m_rumbleType;
  private int m_pulseCount;
  // Change name
  private boolean m_isPulsing;
  private int m_pulses;

  public RumbleCommand(
      GenericHID controller, double intensity, double time, RumbleType rumbleType) {
    m_controller = controller;
    m_intensity = intensity;
    m_rumbleType = rumbleType;
    m_execute = () -> {};
    m_isFinished = () -> m_timer.hasElapsed(time);
  }

  public RumbleCommand(
      GenericHID controller,
      double intensity,
      int pulses,
      double pulseInterval,
      double pulsePause,
      RumbleType rumbleType) {
    m_rumbleType = rumbleType;
    m_controller = controller;
    m_intensity = intensity;
    m_isPulsing = true;
    m_pulses = pulses;
    m_execute =
        () -> {
          if (m_isPulsing) {
            if (m_timer.hasElapsed(pulseInterval)) {
              m_controller.setRumble(m_rumbleType, 0);
              m_timer.reset();
              m_pulseCount++;
              m_isPulsing = false;
            }
          } else {
            if (m_timer.hasElapsed(pulsePause)) {
              m_controller.setRumble(m_rumbleType, m_intensity);
              m_timer.reset();
              m_isPulsing = true;
            }
          }
        };

    m_isFinished = () -> arePulsesCompleted();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_pulseCount = 0;
    m_controller.setRumble(m_rumbleType, m_intensity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_execute.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(m_rumbleType, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished.getAsBoolean();
  }

  private boolean arePulsesCompleted() {
    return m_pulseCount == m_pulses;
  }
}
