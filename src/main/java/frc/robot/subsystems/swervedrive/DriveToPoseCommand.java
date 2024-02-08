// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/** A more advanced Swerve Control System that has 4 buttons for which direction to face */
public class DriveToPoseCommand extends Command {

  private SwerveSubsystem swerve;
  private final double vX, vY;
  private Rotation2d m_desiredHeading;
  private boolean resetHeading = false;

  /**
   * Used to drive a swerve robot in full field-centric mode. vX and vY supply translation inputs,
   * where x is torwards/away from alliance wall and y is left/right. Heading Adjust changes the
   * current heading after being multipied by a constant. The look booleans are shortcuts to get the
   * robot to face a certian direction. Based off of ideas in
   * https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
   *
   * @param swerve The swerve drivebase subsystem.
   * @param vX double that supplies the x-translation joystick input. Should be in the range -1 to 1
   *     with deadband already accounted for. Positive X is away from the alliance wall.
   * @param vY double that supplies the y-translation joystick input. Should be in the range -1 to 1
   *     with deadband already accounted for. Positive Y is towards the left wall when looking
   *     through the driver station glass.
   */
  public DriveToPoseCommand(SwerveSubsystem swerve, double vX, double vY, double desiredHeading) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;

    m_desiredHeading = Rotation2d.fromDegrees(desiredHeading);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    resetHeading = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double headingX = m_desiredHeading.getSin();
    double headingY = m_desiredHeading.getCos();

    // // These are written to allow combinations for 45 angles
    // // Face Away from Drivers
    // if (lookAway) {
    //   headingY = -1;
    // }
    // // Face Right
    // if (lookRight) {
    //   headingX = 1;
    // }
    // // Face Left
    // if (lookLeft) {
    //   headingX = -1;
    // }
    // // Face Towards the Drivers
    // if (lookTowards) {
    //   headingY = 1;
    // }

    // Prevent Movement After Auto
    // if (resetHeading) {
    //   if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust) > 0) {
    //     // Get the curret Heading
    //     Rotation2d currentHeading = swerve.getHeading();

    //     // Set the Current Heading to the desired Heading
    //     headingX = currentHeading.getSin();
    //     headingY = currentHeading.getCos();
    //   }
    //   // Dont reset Heading Again
    //   resetHeading = false;
    // }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX, vY, headingX, headingY);

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation =
        SwerveMath.limitVelocity(
            translation,
            swerve.getFieldVelocity(),
            swerve.getPose(),
            Constants.LOOP_TIME,
            Constants.ROBOT_MASS,
            List.of(Constants.CHASSIS),
            swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // // Make the robot move
    // if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust) > 0) {
    //   resetHeading = true;
    //   swerve.drive(
    //       translation,
    //       (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust),
    //       true);
    // } else {
    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
  }
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
