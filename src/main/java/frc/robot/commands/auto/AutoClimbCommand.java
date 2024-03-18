// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.subsystems.arm.SetArmCommand;
import frc.robot.commands.subsystems.drive.TranslateToPositionCommand;
import frc.robot.commands.subsystems.drive.TurnToAngleCommand;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.preset.Presets;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.utils.AllianceFlipUtil;
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
    List<Pose2d> climbPoses =
        List.of(
            AllianceFlipUtil.apply(
                new Pose2d(4.42, 4.85, new Rotation2d(Units.degreesToRadians(-60)))),
            AllianceFlipUtil.apply(
                new Pose2d(5.8, 4.11, new Rotation2d(Units.degreesToRadians(180)))),
            AllianceFlipUtil.apply(
                new Pose2d(4.42, 3.33, new Rotation2d(Units.degreesToRadians(60)))));
    Pose2d climbPose = drive.getPose().nearest(climbPoses);
    final double backupDistanceMeters = 1;
    addCommands(
        new SequentialCommandGroup(
            new TurnToAngleCommand(
                drive,
                () -> climbPose,
                Constants.Auton.kDriveRotPosSetpointTolerance,
                Constants.Auton.kDriveRotVelSetpointTolerance,
                true),
            new TranslateToPositionCommand(
                drive,
                () ->
                    new Pose2d(
                        climbPose.getX()
                            - backupDistanceMeters * Math.cos(climbPose.getRotation().getRadians()),
                        climbPose.getY()
                            - backupDistanceMeters * Math.sin(climbPose.getRotation().getRadians()),
                        climbPose.getRotation()),
                true),
            new TurnToAngleCommand(
                drive,
                () -> climbPose,
                Constants.Auton.kDriveRotPosSetpointTolerance,
                Constants.Auton.kDriveRotVelSetpointTolerance,
                true),
            new SetArmCommand(armRot, armExt, () -> Presets.kClimbPreset),
            new TranslateToPositionCommand(drive, () -> climbPose, true),
            new SemiAutoClimbCommand(armRot, armExt, climber)));
  }
}
