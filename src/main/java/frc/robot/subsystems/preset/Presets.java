package frc.robot.subsystems.preset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;
import frc.utils.AllianceFlipUtil;
import java.util.*;

public class Presets {
  public enum Preset {
    /**
     * NOTE: ALL PRESET ARE WITH BLUE ALLIANCE VALUES. Getting a Preset pose using {@link
     * #getPose()} will return Pose based on the current alliance.
     */
    kStart("START", 180, 0),
    kStow("STOW", 198, 0),
    kFloorPickup("FLOOR PICKUP", 202, 11),
    kClimbPreset("CLIMB PRESET", 110, 10),
    kClimbReady("CLIMB READY", 110, 7),
    kClimbEnd("CLIMB FINISH", 180, 0),

    kShootAmp(
        "AMP",
        80,
        10,
        1000,
        1000,
        new Pose2d(FieldConstants.ampCenter.getX(), 7.95, Rotation2d.fromDegrees(-90))),
    kShootSubwoofer(
        "SUBWOOFER",
        201,
        0,
        3000,
        5000,
        new Pose2d(
            1.3, FieldConstants.Speaker.centerSpeakerOpening.getY(), Rotation2d.fromDegrees(0))),
    kShootPodium("PODIUM", 180, 0, 2400, 4700, new Pose2d(2.6, 4.3, Rotation2d.fromDegrees(-23.3))),
    kShootStage(
        "STAGE", 163.5, 0, 5000, 5000, new Pose2d(4.85, 4.5, Rotation2d.fromDegrees(-13.5))),
    kLongPass("LONG PASS", 190, 0, 5500, 5500, new Pose2d(10, 1, Rotation2d.fromDegrees(-25))),
    kShootSubwoofers(
        "SUBWOOFER",
        201,
        0,
        3000,
        5000,
        List.of(
            new Pose2d(0.74, 6.6, Rotation2d.fromDegrees(60)),
            new Pose2d(1.3, 5.55, Rotation2d.fromDegrees(0)),
            new Pose2d(0.74, 4.5, Rotation2d.fromDegrees(-60))));

    private double m_rot;
    private double m_ext;
    private int m_topRPM;
    private int m_bottomRPM;
    private Pose2d m_robotPose;
    private String m_string;
    private final List<Pose2d> m_robotPoses;

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
      this.m_robotPoses = null;
    }

    Preset(
        String Name,
        double Rot_Deg,
        double Ext_In,
        int TopRPM,
        int BottomRPM,
        List<Pose2d> robotPoses) {
      this.m_string = Name;
      this.m_rot = Rot_Deg;
      this.m_ext = Ext_In;
      this.m_topRPM = TopRPM;
      this.m_bottomRPM = BottomRPM;
      this.m_robotPose = robotPoses.get(0);
      this.m_robotPoses = robotPoses;
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

    public List<Pose2d> getPoses() {
      List<Pose2d> list = new ArrayList<Pose2d>();
      for (int i = 0; i < 2; i++) {
        list.add(AllianceFlipUtil.apply(m_robotPoses.get(i)));
      }
      return list;
    }

    public String getString() {
      return m_string.toUpperCase();
    }
  }
}
