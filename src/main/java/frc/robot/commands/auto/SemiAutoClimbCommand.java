// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.preset.Presets;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SemiAutoClimbCommand extends SequentialCommandGroup {

  public SemiAutoClimbCommand(
      ArmRotationSubsystem armRot, ArmExtensionSubsystem armExt, ClimberSubsystem climber) {
    setName("Auto Climb Command");
    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(() -> climber.setPower(-1.0)),
            // new WaitCommand(1),
            new SetArmCommand(armRot, armExt, () -> Presets.kClimbEnd),
            new WaitCommand(2),
            new InstantCommand(() -> climber.setPower(0))));
  }
}
