package frc.robot.subsystems.preset;

public class Presets {
  public enum Preset {
    kStart("START", 180, 0, 0, 0),
    kStow("STOW", 200, 0, 0, 0),
    kFloorPickup("FLOOR PICKUP", 203, 11, 500, 500),
    kShootInsidePerimeter("SUBWOOFER INSIDE", 201, 0, 2200, 4700),
    kShootSpeaker_0m("SPEAKER_0M", 198, 10, 4000, 5200),
    kShootSpeaker_2m("SPEAKER_2M", 187, 0, 4500, 4500),
    kShootSpeaker_3m("SPEAKER_3M", 187, 0, 4000, 4000),
    kShootAmp("AMP", 80, 10, 3000, 3000),
    kLongPass("PASS", 170, 0, 5500, 5500),
    kDiscardNote("DISCARD", 185, 0, 0, 5500),
    kStartClimb("START_CLIMB", 90, 10, 0, 0);

    private double m_rot;
    private double m_ext;
    private int m_topRPM;
    private int m_bottomRPM;
    private String m_string;

    Preset(String Name, double Rot_Deg, double Ext_In, int TopRPM, int BottomRPM) {
      this.m_rot = Rot_Deg;
      this.m_ext = Ext_In;
      this.m_topRPM = TopRPM;
      this.m_bottomRPM = BottomRPM;
      this.m_string = Name;
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

    public String getString() {
      return m_string.toUpperCase();
    }
  }
}
