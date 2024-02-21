// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Presets.Preset;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.commands.subsystems.shooter.ShooterCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFromAmpCommand extends SequentialCommandGroup {

  public ShootFromAmpCommand(IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    addCommands(
        new SetArmCommand(arm, Preset.kShootAmp),
        new ShooterCommand(intake, shooter, Preset.kShootAmp));
  }
}
