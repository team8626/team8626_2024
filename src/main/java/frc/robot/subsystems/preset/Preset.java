package frc.robot.subsystems.preset;

import edu.wpi.first.math.geometry.Pose2d;
import frc.utils.AllianceFlipUtil;

public class Preset {
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

  Preset(String Name, double Rot_Deg, double Ext_In, int TopRPM, int BottomRPM, Pose2d robotPose) {
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
   * NOTE: ALL PRESET ARE WITH BLUE ALLIANCE VALUES. Getting a Preset pose using {@link #getPose()}
   * will return Pose based on the current alliance.
   */
  public Pose2d getPose() {
    return AllianceFlipUtil.apply(m_robotPose);
  }

  public String getString() {
    return m_string.toUpperCase();
  }
}
