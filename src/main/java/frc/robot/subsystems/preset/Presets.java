package frc.robot.subsystems.preset;

public class Presets {
  public enum Preset {
    kStart("START", 110, 0, 0, 0),
    kStow("STOW", 195, 0, 0, 0),
    kFloorPickup("FLOOR PICKUP", 204, 10, 500, 500),
    kShootSpeaker_0m("SPEAKER_0M", 204, 10, 4000, 5200),
    kShootSpeaker_2m("SPEAKER_2M", 187, 0, 4000, 4000),
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

    Preset(
        String newString, double newRotDeg, double newExtInches, int newTopRPM, int newBottomRPM) {
      this.m_rot = newRotDeg;
      this.m_ext = newExtInches;
      this.m_topRPM = newTopRPM;
      this.m_bottomRPM = newBottomRPM;
      this.m_string = newString;
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
      return m_string;
    }
  }
}