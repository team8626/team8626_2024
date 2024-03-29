// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.commands.subsystems.shooter.ShooterCommand;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.preset.Presets;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFromSpeakerCommand extends SequentialCommandGroup {

  public ShootFromSpeakerCommand(
      ArmRotationSubsystem armRot,
      ArmExtensionSubsystem armExt,
      IntakeSubsystem intake,
      ShooterSubsystem shooter) {
    addCommands(
        new SetArmCommand(armRot, armExt, () -> Presets.kShootSubwoofer),
        new ShooterCommand(intake, shooter, () -> Presets.kShootSubwoofer));
  }
}
