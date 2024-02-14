// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.commands.SetArmCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.IntakeCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.drivebase.DriveToPoseCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupNoteCommand extends SequentialCommandGroup {

  public PickupNoteCommand(SwerveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
    /* TODO: Once new PID DriveToPose is visible to this branch current pose supplier will work, will just permanently pass starting pose currently */
    addCommands(
        new SetArmCommand(arm, ArmConstants.Presets.kFloorPickup),
        new IntakeCommand(intake)
            .deadlineWith(new DriveToPoseCommand(drive, () -> drive.getPose())));
  }
}
