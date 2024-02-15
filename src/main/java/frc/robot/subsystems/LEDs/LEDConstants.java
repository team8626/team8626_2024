package frc.robot.subsystems.LEDs;

public class LEDConstants {
  public static final int kLEDLength = 75;
  public static final int kLEDPort = 9;

  public static final int[][] kErrorLightIndex = {
    {0, 1, 2, 3, 4, 5}, {20, 21, 22, 23, 24}, {45, 46, 47, 48, 49}, {70, 71, 72, 73, 74}
  };

  public static enum LedMode {
    NOT_CONNECTED,
    DISABLED,
    DEFAULT,
    TEST,
    // CONNECTED,
    // AUTO_FINISHED,
    // AUTONOMOUS,
    // CLIMBING,
    // HANGING,
    // SHOOTING,
    // CONTAINING,
    INTAKING,
    BLUE_ALLIANCE,
    RED_ALLIANCE,
    // LOW_BATTERY_ALERT,
    // DISABLED,
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
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    private final int id;

    errorSections(int id) {
      this.id = id;
    }

    public int getValue() {
      return id;
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
