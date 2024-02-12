// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  private ShooterSubsystem m_shooter;

<<<<<<< HEAD
  private IntakeSubsystem m_intake;

  private int m_speed;

  public ShooterCommand(IntakeSubsystem intake, ShooterSubsystem shooter, int speed) {
=======
  private int m_speed;

  public ShooterCommand(ShooterSubsystem shooter, int speed) {
>>>>>>> 27410ae (Prevent Double Swerve Library Gyro causing crash)
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
    m_shooter = shooter;
    m_intake = intake;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setMotors(m_speed);
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
    return !m_intake.isFull();
  }
}
