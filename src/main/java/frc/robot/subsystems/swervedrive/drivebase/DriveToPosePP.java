// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swervedrive/commands/DriveToPosePPCommand.java
public class DriveToPosePPCommand extends ParallelCommandGroup {
========
public class DriveToPosePP extends ParallelCommandGroup {
  SwerveSubsystem m_drive;
>>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation):src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java

  public DriveToPosePPCommand(SwerveSubsystem drive, Pose2d desiredPose) {

<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swervedrive/commands/DriveToPosePPCommand.java
    setName("Drive To Pose Path Planner Command");
========
  public DriveToPosePP(SwerveSubsystem drive, Pose2d desiredPose) {
    m_drive = drive;
>>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation):src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java

    addCommands(drive.driveToPose(desiredPose));
  }
}
