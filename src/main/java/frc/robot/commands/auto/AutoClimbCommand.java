// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.preset.Presets.Preset;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimbCommand extends SequentialCommandGroup {

  public AutoClimbCommand(
      SwerveSubsystem drive,
      ArmRotationSubsystem armRot,
      ArmExtensionSubsystem armExt,
      ClimberSubsystem climber) {
    setName("Auto Climb Command");
    List<Pose2d> climbPoses = List.of(new Pose2d(), new Pose2d(), new Pose2d());
    Pose2d climbPose = drive.getPose().nearest(climbPoses);
    addCommands(
        new SequentialCommandGroup(
            new SetArmCommand(armRot, armExt, () -> Preset.kClimbPreset),
            // new TurnToAngleCommand(drive, () -> climbPose, false),
            // new TranslateToPositionCommand(drive, climbPose, false),
            // new TurnToAngleCommand(drive, () -> climbPose, false),
            new SetArmCommand(armRot, armExt, () -> Preset.kClimbReady),
            new InstantCommand(() -> climber.setPower(-1.0)),
            new WaitCommand(2),
            new SetArmCommand(armRot, armExt, () -> Preset.kClimbEnd),
            new WaitCommand(1),
            new InstantCommand(() -> climber.setPower(0))));
  }
}
