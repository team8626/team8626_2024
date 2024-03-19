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
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.utils.AllianceFlipUtil;

public class PresetManager implements ImplementDashboard {

  private Preset m_preset;
  private StructPublisher<Pose2d> m_publisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/Presets/Pose2d", Pose2d.struct)
          .publish();

  StructPublisher<Pose2d> publisher_dummy =
      NetworkTableInstance.getDefault().getStructTopic("MyPreset", Pose2d.struct).publish();

  private static double m_ooomf = 2; // m.s-1
  private static double m_angleAdjust = -9;
  private static double m_launchRPMTopMultiplier = 0.65;

  public PresetManager() {
    m_preset = Presets.kShootSubwoofer;

    // SmartDashboard.getNumber("Presets/AimPreset/Launch Angle Adjust (deg)", angleAdjust);
    // SmartDashboard.getNumber("Presets/Ooomf", ooomf);
    // SmartDashboard.getNumber("Presets/AimPreset/TopRPMMultiplier", launchRPMTopMultiplier);
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

    // angleAdjust =
    //     SmartDashboard.getNumber("Presets/AimPreset/Launch Angle Adjust (deg)", angleAdjust);
    // ooomf = SmartDashboard.getNumber("Presets/Ooomf", ooomf);
    // launchRPMTopMultiplier =
    //     SmartDashboard.getNumber("Presets/AimPreset/TopRPMMultiplier", launchRPMTopMultiplier);

    double maxArmRotationInsideFrame = 201; /* Degrees */
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

    // double ooomf = 0; // m.s-1

    double vZ = Math.sqrt((m_ooomf * m_ooomf) + (targetHeight - z0) * 2 * 9.81);
    double tm = (vZ - m_ooomf) / 9.81;
    double vX = targetDistance / tm;

    // double angleAdjust = -9;
    // double launchRPMTopMultiplier = 0.6;

    Rotation2d launchAngle = new Rotation2d(Math.atan(vZ / vX));
    Rotation2d armAngle =
        new Rotation2d(
            Units.degreesToRadians(
                launchAngle.getDegrees()
                    + 180
                    - 30 /* Horizontal arm: 180deg, Shooter/Arm: -30deg */
                    + m_angleAdjust)); /* Adjust after tuning */

    if (armAngle.getDegrees() > maxArmRotationInsideFrame) {
      armAngle = new Rotation2d(Units.degreesToRadians(maxArmRotationInsideFrame));
    }

    double launchVelocity = Math.sqrt((vX * vX) + (vZ * vZ));
    double launchRPM =
        Math.min(
            (launchVelocity / (Math.PI * ShooterConstants.kFlywheelDiameterMeters / 2)) * 60,
            ShooterConstants.kMaxRPM);

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
        (int) (launchRPM * m_launchRPMTopMultiplier),
        (int) launchRPM,
        new Pose2d(robotX, robotY, robotRotation));
  }

  public static Pose2d getClosedClimbingStart(Pose2d robotPose) {
    return AllianceFlipUtil.apply(robotPose.nearest(Presets.kClimbPoses));
  }

  @Override
  public void initDashboard() {
    SmartDashboard.putNumber("Presets/AimPreset/Launch Angle Adjust (deg)", m_angleAdjust);
    SmartDashboard.putNumber("Presets/Ooomf", m_ooomf);
    SmartDashboard.putNumber("Presets/AimPreset/TopRPMMultiplier", m_launchRPMTopMultiplier);
  }

  @Override
  public void updateDashboard() {
    m_publisher.set(m_preset.getPose());
    publisher_dummy.set(m_preset.getPose());
    SmartDashboard.putString("Presets/Preset", m_preset.getString());

    m_angleAdjust =
        SmartDashboard.getNumber("Presets/AimPreset/Launch Angle Adjust (deg)", m_angleAdjust);
    m_ooomf = SmartDashboard.getNumber("Presets/Ooomf", m_ooomf);
    m_launchRPMTopMultiplier =
        SmartDashboard.getNumber("Presets/AimPreset/TopRPMMultiplier", m_launchRPMTopMultiplier);
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }
}
