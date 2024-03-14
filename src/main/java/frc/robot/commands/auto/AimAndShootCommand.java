// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystems.drive.TurnToAngleCommand;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.preset.PresetManager;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AimAndShootCommand extends SequentialCommandGroup {

  public AimAndShootCommand(SwerveSubsystem drive) { // }, Supplier<Preset> desiredState) {
    setName("AimAndShootCommand");

    addCommands(
        new InstantCommand(() -> LEDSubsystem.setMode(LedMode.AUTOSHOOT)),
        new TurnToAngleCommand(
            drive, () -> PresetManager.getAimAndShootPreset(drive.getPose()).getPose(), true),
        new InstantCommand(() -> LEDSubsystem.setMode(LedMode.DEFAULT)));
  }
}
