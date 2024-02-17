package frc.robot;

public class SubsystemsConstants {
  public static enum Preset {
    kStart(110, 0, 500, 500),
    kStow(195, 0, 500, 500),
    kFloorPickup(20, 5, 500, 500),
    kShootSpeaker_0m(195, 0, 4000, 5500),
    kShootSpeaker_2m(20, 0, 500, 500),
    kShootSpeaker_3m(20, 0, 500, 500),
    kShootAmp(80, 6, 1000, 1000),
    kStartClimb(90, 10, 500, 500);

    private final double m_rot;
    private final double m_ext;
    private final int m_topRPM;
    private final int m_bottomRPM;

    Preset(double newRotDeg, double newExtInches, int newTopRPM, int newBottomRPM) {
      this.m_rot = newRotDeg;
      this.m_ext = newExtInches;
      this.m_topRPM = newTopRPM;
      this.m_bottomRPM = newBottomRPM;
    }

    public double getRotDegrees() {
      return m_rot;
    }

    public double getExtInches() {
      return m_ext;
    }

    public double getTopRPM() {
      return m_topRPM;
    }

    public double getBottomRPM() {
      return m_bottomRPM;
    }
  }
}
