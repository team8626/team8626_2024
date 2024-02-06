package frc.robot.subsystems.LEDs;

public class LEDConstants {
  public static final int kLEDLength = 30;
  public static final int kLEDPort = 0;
  public static final int[] kErrorLightIndex = {0, 1, 2, 3, 4, 25, 26, 27, 28, 29};

  public static final int minLoopCycleCount = 10;
  public static final int length = 10;
  public static final double strobeFastDuration = 0.1;
  public static final double strobeSlowDuration = 0.2;
  public static final double breathDuration = 1.0;
  public static final double rainbowCycleLength = 25.0;
  public static final double rainbowDuration = 0.25;
  public static final double waveExponent = 0.4;
  public static final double waveFastCycleLength = 25.0;
  public static final double waveFastDuration = 0.25;
  public static final double waveSlowCycleLength = 25.0;
  public static final double waveSlowDuration = 3.0;
  public static final double waveAllianceCycleLength = 15.0;
  public static final double waveAllianceDuration = 2.0;
  public static final double autoFadeTime = 2.5; // 3s nominal
  public static final double autoFadeMaxTime = 5.0;
}
