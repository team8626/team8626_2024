package frc.robot.subsystems.LEDs;

public class LEDConstants {
  public static final int kLEDStripLength = 131;
  public static final int kLEDPort = 9;

  // public static final int[][] kErrorLightIndex = {
  //   {0, 1, 2, 3, 4}, {20, 21, 22, 23, 24}, {45, 46, 47, 48, 49}, {70, 71, 72, 73, 74}
  // };

  record LEDSection(int startId, int endId) {}

  public static final LEDSection kSectionMain = new LEDSection(0, 74);
  public static final LEDSection kSectionAmbience = new LEDSection(75, 130);

  public static enum LedMode {
    NOT_CONNECTED,
    DISABLED,
    DEFAULT,
    PRESHOOTING,
    SHOOTING,
    FOLLOWNOTE,
    INTAKING,
    AMPLIFICATION,
    BLUE_ALLIANCE,
    RED_ALLIANCE,
    OFF
  }

  public static enum LedAmbienceMode {
    RAINBOW,
    OFF
  }

  public static enum LedErrorMode {
    NO_ERROR,
    ERROR_ESTOP,
    ERROR_CRITICAL,
    ERROR_DRIVE_FL,
    ERROR_DRIVE_FR,
    ERROR_DRIVE_BL,
    ERROR_DRIVE_BR
  }

  public static enum errorSections {
    FRONT_RIGHT(0, new int[] {0, 1, 2, 3, 4}),
    BACK_RIGHT(1, new int[] {20, 21, 22, 23, 24}),
    BACK_LEFT(2, new int[] {45, 46, 47, 48, 49}),
    FRONT_LEFT(3, new int[] {70, 71, 72, 73, 74});

    private final int id;
    private final int[] indexes;

    errorSections(int id, int[] indexes) {
      this.id = id;
      this.indexes = indexes;
    }

    public int getValue() {
      return id;
    }

    public int[] getIndexes() {
      return indexes;
    }
  }

  public static final int minLoopCycleCount = 10;
  public static final int length = 10;
  public static final double strobeFastDuration = 0.1;
  public static final double strobeSlowDuration = 0.2;
  public static final double breathDuration = 2.0;
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
