package frc.robot.subsystems.preset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;

public class Presets {
  /**
   * NOTE: ALL PRESET ARE WITH BLUE ALLIANCE VALUES. Getting a Preset pose using {@link #getPose()}
   * will return Pose based on the current alliance.
   */
  public static final Preset kStart = new Preset("START", 180, 0);

  public static final Preset kStow = new Preset("STOW", 198, 0);
  public static final Preset kFloorPickup = new Preset("FLOOR PICKUP", 202, 11);
  public static final Preset kClimbPreset = new Preset("CLIMB PRESET", 110, 10);
  public static final Preset kClimbReady = new Preset("CLIMB READY", 110, 7);
  public static final Preset kClimbEnd = new Preset("CLIMB FINISH", 180, 0);

  public static final Preset kShootAmp =
      new Preset(
          "AMP",
          80,
          10,
          1000,
          1000,
          new Pose2d(FieldConstants.ampCenter.getX(), 7.95, Rotation2d.fromDegrees(-90)));
  public static final Preset kShootSubwoofer =
      new Preset(
          "SUBWOOFER",
          201,
          0,
          3000,
          5000,
          new Pose2d(
              1.3, FieldConstants.Speaker.centerSpeakerOpening.getY(), Rotation2d.fromDegrees(0)));

  public static final Preset kShootPodium =
      new Preset("PODIUM", 180, 0, 2400, 4700, new Pose2d(2.6, 4.3, Rotation2d.fromDegrees(-23.3)));

  public static final Preset kShootStage =
      new Preset(
          "STAGE", 163.5, 0, 5000, 5000, new Pose2d(4.85, 4.5, Rotation2d.fromDegrees(-13.5)));
  public static final Preset kLongPass =
      new Preset("LONG PASS", 190, 0, 5500, 5500, new Pose2d(10, 1, Rotation2d.fromDegrees(-25)));
}
