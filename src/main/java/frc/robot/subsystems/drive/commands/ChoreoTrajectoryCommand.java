// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import com.choreo.lib.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Optional;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChoreoTrajectoryCommand extends ParallelCommandGroup {

  DriveSubsystem m_drive;

  public ChoreoTrajectoryCommand(DriveSubsystem drive) {

    ChoreoTrajectory
        traj; // Here I want to create a trajectory without having known timepoints with states

    m_drive = drive;
    traj = Choreo.getTrajectory("Trajectory");
    Command trajCommand =
        Choreo.choreoSwerveCommand(
            traj,
            m_drive::getPose,
            new PIDController(DriveConstants.AutoConstants.kPXController, 0.0, 0.0),
            new PIDController(DriveConstants.AutoConstants.kPXController, 0.0, 0.0),
            new PIDController(DriveConstants.AutoConstants.kPThetaController, 0.0, 0.0),
            (ChassisSpeeds speeds) ->
                m_drive.drive(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond,
                    false,
                    false),
            () -> {
              Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
              boolean mirror = alliance.isPresent() && alliance.get() == Alliance.Red;
              return mirror;
            },
            m_drive);
    addCommands(trajCommand);
    addRequirements(drive);
  }
}
