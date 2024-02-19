// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Presets.Preset;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.commands.SetArmCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ShooterCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFromSpeakerCommand extends SequentialCommandGroup {

  public ShootFromSpeakerCommand(
      IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    addCommands(
        new SetArmCommand(arm, Preset.kShootSpeaker_0m),
        new ShooterCommand(intake, shooter, Preset.kShootSpeaker_0m));
  }
}
