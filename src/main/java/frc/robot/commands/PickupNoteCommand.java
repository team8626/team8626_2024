// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.commands.SetArmCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.IntakeCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.commands.PoseDisplacementCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupNoteCommand extends SequentialCommandGroup {

  public PickupNoteCommand(SwerveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake) {
    addCommands(
        new SetArmCommand(arm, ArmConstants.Presets.kFloorPickup),
        new IntakeCommand(intake)
            .deadlineWith(
                new PoseDisplacementCommand(
                    drive, new Transform2d(new Translation2d(1, 0), new Rotation2d()), true)));
  }
}
