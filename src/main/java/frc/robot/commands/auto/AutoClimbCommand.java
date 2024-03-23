// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
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
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimbCommand extends SequentialCommandGroup {

  private Pose2d m_climbPose;

  public AutoClimbCommand(
      SwerveSubsystem drive,
      ArmRotationSubsystem armRot,
      ArmExtensionSubsystem armExt,
      ClimberSubsystem climber,
      Supplier<Pose2d> climbPoseSupplier) {
    setName("Auto Climb Command");

    m_climbPose = climbPoseSupplier.get();
    final double backupDistanceMeters = 1;
    addCommands(
        new SequentialCommandGroup(
            new TurnToAngleCommand(
                drive,
                climbPoseSupplier,
                Units.degreesToRadians(4),
                Constants.Auton.kDriveRotVelSetpointTolerance,
                true),
            new TranslateToPositionCommand(
                // new DriveToPoseTrajPIDCommand(
                drive,
                () ->
                    new Pose2d(
                        climbPoseSupplier.get().getX()
                            - backupDistanceMeters
                                * Math.cos(climbPoseSupplier.get().getRotation().getRadians()),
                        climbPoseSupplier.get().getY()
                            - backupDistanceMeters
                                * Math.sin(climbPoseSupplier.get().getRotation().getRadians()),
                        climbPoseSupplier.get().getRotation()),
                true),
            // new TurnToAngleCommand(
            //     drive,
            //     climbPoseSupplier,
            //     Constants.Auton.kDriveRotPosSetpointTolerance,
            //     Constants.Auton.kDriveRotVelSetpointTolerance,
            //     true),
            new SetArmCommand(armRot, armExt, () -> Presets.kClimbPreset),
            new TranslateToPositionCommand(drive, () -> climbPoseSupplier.get(), true),
            new SemiAutoClimbCommand(armRot, armExt, climber)));
  }
}
