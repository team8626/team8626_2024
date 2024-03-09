package frc.robot.subsystems.preset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;
import frc.utils.AllianceFlipUtil;

public class Presets {
  public enum Preset {
    /**
     * NOTE: ALL PRESET ARE WITH BLUE ALLIANCE VALUES. Getting a Preset pose using {@link
     * #getPose()} will return Pose based on the current alliance.
     */
    kStart("START", 180, 0),
    kStow("STOW", 198, 0),
    kFloorPickup("FLOOR PICKUP", 202, 11),
    kClimbPreset("CLIMB PRESET", 110, 8),
    kClimbReady("CLIMB READY", 130, 5),
    kClimbEnd("CLIMB FINISH", 180, 0),

    kShootAmp(
        "AMP",
        80,
        10,
        3000,
        3000,
        new Pose2d(FieldConstants.ampCenter.getX(), 7.82, Rotation2d.fromDegrees(-90))),
    kShootSubwoofer(
        "SUBWOOFER",
        201,
        0,
        3000,
        5000,
        new Pose2d(
            1.3, FieldConstants.Speaker.centerSpeakerOpening.getY(), Rotation2d.fromDegrees(0))),
    kShootPodium("PODIUM", 180, 0, 4000, 5200, new Pose2d(2.6, 4.3, Rotation2d.fromDegrees(-23.3))),
    kShootStage(
        "STAGE", 163.5, 0, 5000, 5000, new Pose2d(4.85, 4.5, Rotation2d.fromDegrees(-13.5))),
    kLongPass("LONG PASS", 190, 0, 5500, 5500, new Pose2d(10, 1, Rotation2d.fromDegrees(-25)));

    private double m_rot;
    private double m_ext;
    private int m_topRPM;
    private int m_bottomRPM;
    private Pose2d m_robotPose;
    private String m_string;

    Preset(String Name, double Rot_Deg, double Ext_In) {
      this(Name, Rot_Deg, Ext_In, 0, 0);
    }

    Preset(String Name, double Rot_Deg, double Ext_In, int TopRPM, int BottomRPM) {
      this(Name, Rot_Deg, Ext_In, 0, 0, new Pose2d());
    }

    Preset(
        String Name, double Rot_Deg, double Ext_In, int TopRPM, int BottomRPM, Pose2d robotPose) {
      this.m_string = Name;
      this.m_rot = Rot_Deg;
      this.m_ext = Ext_In;
      this.m_topRPM = TopRPM;
      this.m_bottomRPM = BottomRPM;
      this.m_robotPose = robotPose;
    }

    public double getRotDegrees() {
      return m_rot;
    }

    public double getExtInches() {
      return m_ext;
    }

    public int getTopRPM() {
      return m_topRPM;
    }

    public int getBottomRPM() {
      return m_bottomRPM;
    }

    /**
     * NOTE: ALL PRESET ARE WITH BLUE ALLIANCE VALUES. Getting a Preset pose using {@link
     * #getPose()} will return Pose based on the current alliance.
     */
    public Pose2d getPose() {
      return AllianceFlipUtil.apply(m_robotPose);
    }

    public String getString() {
      return m_string.toUpperCase();
    }
  }
}
