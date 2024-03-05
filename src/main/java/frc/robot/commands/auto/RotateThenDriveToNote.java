package frc.robot.commands.auto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateThenDriveToNote extends SequentialCommandGroup {

  public RotateThenDriveToNote(SwerveSubsystem drive, IntakeSubsystem intake) {
    setName("Rotate Then Drive To Note");
    addCommands(
        new SequentialCommandGroup(new RotateToNoteCommand(drive)),
        new DriveToNoteCommand(drive, intake));
  }
}
