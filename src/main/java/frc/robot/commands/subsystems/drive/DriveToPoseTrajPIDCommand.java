// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPoseTrajPIDCommand extends SequentialCommandGroup {

  public DriveToPoseTrajPIDCommand(
      SwerveSubsystem drive, Supplier<Pose2d> desiredPoseSupplier, boolean lockPose) {
    Supplier<Pose2d> m_desiredPoseSupplier = desiredPoseSupplier;
    Supplier<Pose2d> finalPPPoseSupplier =
        () -> {
          Pose2d desiredPose = desiredPoseSupplier.get();

          // Only works with deffered command
          boolean xDisplacementPositive = (desiredPose.getX() - drive.getPose().getX()) > 0;
          boolean yDisplacementPositive = (desiredPose.getY() - drive.getPose().getY()) > 0;
          Pose2d correctedPPPose =
              new Pose2d(
                  xDisplacementPositive ? desiredPose.getX() - 0.5 : desiredPose.getX() + 0.5,
                  yDisplacementPositive ? desiredPose.getY() - 0.5 : desiredPose.getY() + 0.5,
                  desiredPose.getRotation());
          return correctedPPPose;
        };

    addCommands(
        new InstantCommand(() -> LEDSubsystem.setMode(LedMode.DRIVETOPOSE)),
        new TurnToAngleCommand(
            drive,
            m_desiredPoseSupplier,
            Constants.Auton.kDriveRotPosSetpointTolerance + 2,
            Constants.Auton.kDriveRotVelSetpointTolerance + 2,
            true),
        new InstantCommand(() -> drive.driveToPose(finalPPPoseSupplier.get()).schedule()),
        new TranslateToPositionCommand(drive, m_desiredPoseSupplier, true),
        new TurnToAngleCommand(
            drive,
            m_desiredPoseSupplier,
            Constants.Auton.kDriveRotPosSetpointTolerance,
            Constants.Auton.kDriveRotVelSetpointTolerance,
            true),
        new InstantCommand(() -> LEDSubsystem.setMode(LedMode.DEFAULT)),
        new InstantCommand(() -> drive.lock()).onlyIf(() -> lockPose));

    setName("Drive To Pose Trajectory and PID Command");
  }

  public DriveToPoseTrajPIDCommand(SwerveSubsystem drive, Pose2d desiredPose, boolean lockPose) {

    addCommands(
        new PrintCommand("(" + desiredPose.getX() + ", " + desiredPose.getY() + ")"),
        new InstantCommand(() -> LEDSubsystem.setMode(LedMode.DRIVETOPOSE)),
        new DriveToPosePPCommand(drive, desiredPose),
        new TranslateToPositionCommand(drive, () -> desiredPose, true),
        new TurnToAngleCommand(drive, () -> desiredPose, Constants.Auton.kDriveRotPosSetpointTolerance, Constants.Auton.kDriveRotVelSetpointTolerance, true),
        new InstantCommand(() -> LEDSubsystem.setMode(LedMode.DEFAULT)),
        new InstantCommand(() -> drive.lock()).onlyIf(() -> lockPose));

    setName("Drive To Pose Trajectory and PID Command");
  }
}
