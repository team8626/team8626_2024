// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.preset.Presets;
import frc.robot.subsystems.swervedrive.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class EasyAmpCommand extends Command {

  private SwerveSubsystem m_drive;
  private CommandXboxController m_xboxController;
  private RobotContainer m_robotContainer;

  private double m_rotPID_P = 5;
  private double m_rotPID_I = 0.75;
  private double m_rotPID_D = 0.25;
  private double m_rotMaxVelocityRadPerSec = Constants.Auton.kMaxAngularSpeedRadiansPerSecond;
  private double m_rotMaxAccelerationRadPerSecSqr =
      Constants.Auton.kMaxAngularSpeedRadiansPerSecondSquared;
  private double m_rotToleranceAngleDeg = 1;
  private double m_rotToleranceVelocityDegPerSec = 1;

  private final ProfiledPIDController m_rotPIDController =
      new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  ;

  // private Supplier<Pose2d> m_desiredPoseSupplier;
  // private double rotDesiredPos;

  private boolean SHOOT = false;

  public EasyAmpCommand(
      SwerveSubsystem drive, CommandXboxController xboxController, RobotContainer robotContainer) {

    m_drive = drive;
    m_xboxController = xboxController;
    m_robotContainer = robotContainer;

    addRequirements(m_drive);
    setName("EasyAmpCommand");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotPIDController.reset(m_drive.getOdometryHeading().getRadians());
    m_rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotPIDController.setPID(m_rotPID_P, m_rotPID_I, m_rotPID_D);
    m_rotPIDController.setTolerance(
        Units.degreesToRadians(m_rotToleranceAngleDeg),
        Units.degreesToRadians(m_rotToleranceVelocityDegPerSec));
    m_rotPIDController.setConstraints(
        new TrapezoidProfile.Constraints(
            Math.toDegrees(m_rotMaxVelocityRadPerSec),
            Math.toDegrees(m_rotMaxAccelerationRadPerSecSqr)));

    LEDSubsystem.setMode(LedMode.DRIVETOPOSE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Adjust Rotation
    m_drive.drive(
        new Translation2d(
            Math.pow(
                    MathUtil.applyDeadband(
                        -m_xboxController.getLeftY() * m_robotContainer.invert, 0.1),
                    3)
                * 6.36,
            Math.pow(
                    MathUtil.applyDeadband(
                        -m_xboxController.getLeftX() * m_robotContainer.invert, 0.1),
                    3)
                * 6.36),
        m_rotPIDController.calculate(
            m_drive.getOdometryHeading().getRadians(),
            Presets.kShootAmp.getPose().getRotation().getRadians()),
        true);

    if (m_xboxController.rightTrigger().getAsBoolean()) {
      SHOOT = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.printf("[EasyAmp] Ended\n");
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return SHOOT;
  }
}
