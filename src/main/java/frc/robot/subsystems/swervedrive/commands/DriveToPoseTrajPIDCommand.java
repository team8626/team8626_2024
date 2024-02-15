// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive.commands;

<<<<<<< HEAD
<<<<<<< HEAD
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
=======
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
>>>>>>> a20f000 (Command Templates)
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPoseTrajPIDCommand extends SequentialCommandGroup {

<<<<<<< HEAD
<<<<<<< HEAD
  public DriveToPoseTrajPIDCommand(SwerveSubsystem drive, Pose2d desiredPose, boolean lockPose) {

    addCommands(
        new DriveToPosePPCommand(drive, desiredPose),
        new DriveToPoseCommand(drive, desiredPose, true),
        new InstantCommand(() -> drive.lock()).onlyIf(() -> lockPose));
=======
  public DriveToPoseTrajPIDCommand(
      SwerveSubsystem drive, double desiredXPos, double desiredYPos, double desiredRot) {

    addCommands(
        new DriveToPosePPCommand(drive, desiredXPos, desiredYPos, desiredRot),
        new DriveToPoseCommand(drive, desiredXPos, desiredYPos, desiredRot, true));
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
=======
  public DriveToPoseTrajPIDCommand(SwerveSubsystem drive, Pose2d desiredPose, boolean lockPose) {

    addCommands(
        new DriveToPosePPCommand(drive, desiredPose),
        new DriveToPoseCommand(drive, desiredPose, true),
        new InstantCommand(() -> drive.lock()).onlyIf(() -> lockPose));
>>>>>>> a20f000 (Command Templates)

    setName("Drive To Pose Trajectory and PID Command");
  }
}
