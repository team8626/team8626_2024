// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.preset.Presets.Preset;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpinShootStowCommand extends SequentialCommandGroup {
  /** Creates a new SpinShootStowCommand. */
  public SpinShootStowCommand(
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      ArmRotationSubsystem armRot,
      ArmExtensionSubsystem armExt,
      Supplier<Preset> preset) {

    addCommands(
        new SpinAndShootCommand(intake, shooter, armRot, armExt, preset),
        new SetArmCommand(armRot, armExt, () -> Preset.kStow));
  }
}
