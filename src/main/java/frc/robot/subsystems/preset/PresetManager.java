// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.preset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
// import frc.robot.Presets.Preset;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.utils.AllianceFlipUtil;

public class PresetManager implements ImplementDashboard {
  private Preset m_preset;
  StructPublisher<Pose2d> m_publisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Preset/Pose2d", Pose2d.struct)
          .publish();

  public PresetManager() {
    m_preset = Presets.kShootSubwoofer;
  }

  public void set(Preset newPreset) {
    m_preset = newPreset;
  }

  public Preset get() {
    return m_preset;
  }

  public Pose2d getPose() {
    return m_preset.getPose();
  }

  public static Preset getAimAndShootPreset(Pose2d robotPose) {

    double targetX = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getX();
    double targetY = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).getY();

    double robotX = robotPose.getX();
    double robotY = robotPose.getY();
    double z0 = Units.inchesToMeters(20); // Shooting Height

    double targetHeight = Units.inchesToMeters(80.5);
    double targetDistance =
        Math.sqrt(
            (targetX - robotX) * (targetX - robotX) + (targetY - robotY) * (targetY - robotY));

    Rotation2d targetElevation =
        new Rotation2d(Math.atan((FieldConstants.topRightSpeaker.getZ() / targetDistance)));

    Rotation2d targetRotation =
        AllianceFlipUtil.apply(
            new Rotation2d(Math.acos((targetY - robotY) / targetDistance) - Math.PI / 2));

    Rotation2d robotRotation = AllianceFlipUtil.apply(targetRotation);

    double ooomf = 0;
    double vZ = Math.sqrt((ooomf * ooomf) + (targetHeight - z0) * 2 * 9.81);
    double tm = (vZ - ooomf) / 9.81;
    double vX = targetDistance / tm;

    Rotation2d launchAngle = new Rotation2d(Math.atan(vZ / vX));
    Rotation2d armAngle =
        new Rotation2d(
            Units.degreesToRadians(
                launchAngle.getDegrees()
                    + 180
                    - 30)); /* Horizontal arm: 180deg, Shooter/Arm: -30deg */

    double launchVelocity = Math.sqrt((vX * vX) + (vZ * vZ));
    double launchRPM =
        (launchVelocity / (Math.PI * ShooterConstants.kFlywheelDiameterMeters / 2)) * 60;

    SmartDashboard.putNumber("Presets/AimPreset/Robot X", robotX);
    SmartDashboard.putNumber("Presets/AimPreset/Robot Y", robotY);
    SmartDashboard.putNumber("Presets/AimPreset/Shooter Z (z0)", z0);

    SmartDashboard.putNumber("Presets/AimPreset/Target X", targetX);
    SmartDashboard.putNumber("Presets/AimPreset/Target Y", targetY);
    SmartDashboard.putNumber("Presets/AimPreset/Target Z", targetHeight);

    SmartDashboard.putNumber("Presets/AimPreset/v0", targetHeight);
    SmartDashboard.putNumber("Presets/AimPreset/Target Z", targetHeight);

    SmartDashboard.putNumber("Presets/AimPreset/Target Distance (m)", targetDistance);

    SmartDashboard.putNumber("Presets/AimPreset/vX", vX);
    SmartDashboard.putNumber("Presets/AimPreset/vZ", vZ);
    SmartDashboard.putNumber("Presets/AimPreset/tM", tm);

    SmartDashboard.putNumber(
        "Presets/AimPreset/Target Rotation (deg)", targetRotation.getDegrees());
    SmartDashboard.putNumber(
        "Presets/AimPreset/Target Elevation (deg)", targetElevation.getDegrees());
    SmartDashboard.putNumber("Presets/AimPreset/Launch Angle (deg)", launchAngle.getDegrees());
    SmartDashboard.putNumber("Presets/AimPreset/Launch Velocity (m.s-1)", launchVelocity);
    SmartDashboard.putNumber("Presets/AimPreset/Launch Speed (RPM)", launchRPM);
    SmartDashboard.putNumber("Presets/AimPreset/Launch Arm Angle (deg)", armAngle.getDegrees());
    SmartDashboard.putNumber(
        "Presets/AimPreset/Robot Rotation Angle (deg)", robotRotation.getDegrees());

    return new Preset(
        "AUTO AIM",
        armAngle.getDegrees(),
        0.0,
        (int) launchRPM,
        (int) launchRPM,
        new Pose2d(robotX, robotY, robotRotation));
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
    m_publisher.set(m_preset.getPose());
    SmartDashboard.putString("Preset/Preset", m_preset.getString());
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
