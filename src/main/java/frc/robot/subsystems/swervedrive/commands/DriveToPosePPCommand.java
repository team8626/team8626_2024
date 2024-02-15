// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive.commands;

import edu.wpi.first.math.geometry.Pose2d;
<<<<<<< HEAD
=======
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java
========
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
>>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template):src/main/java/frc/robot/subsystems/swervedrive/commands/DriveToPosePPCommand.java
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
<<<<<<< HEAD
=======
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swervedrive/commands/DriveToPosePPCommand.java
public class DriveToPosePPCommand extends ParallelCommandGroup {
========
public class DriveToPosePP extends ParallelCommandGroup {
<<<<<<< HEAD
=======
========
public class DriveToPosePPCommand extends ParallelCommandGroup {
>>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template):src/main/java/frc/robot/subsystems/swervedrive/commands/DriveToPosePPCommand.java
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
  SwerveSubsystem m_drive;
>>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation):src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java

  public DriveToPosePPCommand(SwerveSubsystem drive, Pose2d desiredPose) {

<<<<<<< HEAD
=======
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swervedrive/commands/DriveToPosePPCommand.java
    setName("Drive To Pose Path Planner Command");
========
  public DriveToPosePP(SwerveSubsystem drive, Pose2d desiredPose) {
<<<<<<< HEAD
    m_drive = drive;
>>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation):src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java

    addCommands(drive.driveToPose(desiredPose));
=======
========
  public DriveToPosePPCommand(
      SwerveSubsystem drive, double desiredXPos, double desiredYPos, double desiredRot) {
>>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template):src/main/java/frc/robot/subsystems/swervedrive/commands/DriveToPosePPCommand.java
    m_drive = drive;
>>>>>>>> 80c6ebd (Untested YAGSL Drive To Pose Implementation):src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java

<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/swervedrive/drivebase/DriveToPosePP.java
    addCommands(drive.driveToPose(desiredPose));
========
    setName("Drive To Pose Path Planner Command");
    addRequirements(m_drive);
    m_driveToPoseCommand =
        m_drive.driveToPose(
            new Pose2d(
                new Translation2d(desiredXPos, desiredYPos),
                Rotation2d.fromDegrees(SwerveSubsystem.convertAngle(desiredRot))));
    addCommands(m_driveToPoseCommand);
>>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template):src/main/java/frc/robot/subsystems/swervedrive/commands/DriveToPosePPCommand.java
>>>>>>> c62b5f9 (Drive To Pose Trajectory and PID Command Template)
  }
}
