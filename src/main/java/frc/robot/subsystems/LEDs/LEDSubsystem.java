// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDConstants.LEDSection;
import frc.robot.subsystems.LEDs.LEDConstants.LedAmbienceMode;
import frc.robot.subsystems.LEDs.LEDConstants.LedErrorMode;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDConstants.errorSections;
import java.util.Optional;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private static AddressableLED m_LEDs;

  private static AddressableLEDBuffer m_buffer;

  static LedMode m_mode = LedMode.OFF;
  static LedErrorMode m_error_mode = LedErrorMode.NO_ERROR;
  static LedAmbienceMode m_ambience_mode = LedAmbienceMode.OFF;

  static Color m_currentColor[] = {Color.kHotPink, Color.kPink};

  public LEDSubsystem() {

    m_LEDs = new AddressableLED(LEDConstants.kLEDPort);
    m_buffer = new AddressableLEDBuffer(LEDConstants.kLEDStripLength);
    m_LEDs.setLength(LEDConstants.kLEDStripLength);
    m_LEDs.setData(m_buffer);
    m_LEDs.start();

    setMode(LedMode.NOT_CONNECTED);
    setAmbienceMode(LedAmbienceMode.OFF);
    setErrorMode(LedErrorMode.NO_ERROR);
  }

  public static void setMode(LedMode newMode) {
    m_mode = newMode;
    updateMainLeds();
  }

  public static void setErrorMode(LedErrorMode newErrorCode) {
    m_error_mode = newErrorCode;
    updateErrorLeds();
  }

  public static void setAmbienceMode(LedAmbienceMode newAmbienceCode) {
    m_ambience_mode = newAmbienceCode;
    updateAmbienceLeds();
  }

  /**
   * Set color of the complete LED Buffer to color
   *
   * @param color
   */
  private void setColor(LEDSection section, Color color) {
    for (int i = section.startId(); i <= section.endId(); i++) {
      m_buffer.setLED(i, color);
    }
  }

  /**
   * Set color of the all Error LEDs
   *
   * @param color
   */
  private static void error(Color color) {
    for (errorSections section : errorSections.values()) {
      error(section, color);
    }
  }

  /**
   * Set color of the specified Error Section
   *
   * @param section
   * @param color
   */
  public static void error(errorSections section, Color color) {
    boolean on = ((Timer.getFPGATimestamp() % 2) / 2) > 0.5;
    for (int j = 0; j < section.getIndexes().length; j++) {
      int ledIndex = section.getIndexes()[j];
      if (on) m_buffer.setLED(ledIndex, color);
      else m_buffer.setLED(ledIndex, Color.kBlack);
    }
  }

  /**
   * Set color of the specified Error Sections
   *
   * @param sections
   * @param color
   */
  private void error(errorSections[] sections, Color color) {
    for (int i = 0; i < sections.length; i++) {
      this.error(sections[i], color);
    }
  }

  /** Update LEDs based on current set state */
  private static void updateMainLeds() {
    switch (m_mode) {
      case NOT_CONNECTED:
        pulse(LEDConstants.kSectionMain, m_currentColor[0], 0.5, 2.0);
        break;

      case DISABLED:
        breath(
            LEDConstants.kSectionMain,
            m_currentColor[0],
            m_currentColor[1],
            LEDConstants.breathDuration,
            Timer.getFPGATimestamp());
        break;

      case PRESHOOTING:
        flow(LEDConstants.kSectionMain, m_currentColor[0], 0.25);
        break;

      case INTAKING:
      case SHOOTING:
        flow(LEDConstants.kSectionMain, m_currentColor[0], 0.5);
        break;

      case FOLLOWNOTE:
        flow(LEDConstants.kSectionMain, Color.kOrange, 1);
        break;

      case AMPLIFICATION:
        blink(LEDConstants.kSectionMain, Color.kLime, .25);
        break;

      case DEFAULT:
      default:
        wave(LEDConstants.kSectionMain, m_currentColor[0], m_currentColor[1], 25, 2.0);
        break;
    }
  }

  /** Update LEDs based on current set state */
  private static void updateAmbienceLeds() {
    switch (m_ambience_mode) {
      case OFF:
        solid(LEDConstants.kSectionAmbience, Color.kBlack);
        break;

      case RAINBOW:
      default:
        rainbow(LEDConstants.kSectionAmbience, 25, 2.0);
        break;
    }
  }

  /** Update ErrorLEDs based on current set state */
  private static void updateErrorLeds() {

    switch (m_error_mode) {
      case ERROR_CRITICAL:
        error(Color.kYellowGreen);
        break;
      case ERROR_DRIVE_FL:
        error(errorSections.FRONT_LEFT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_FR:
        error(errorSections.FRONT_RIGHT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_BL:
        error(errorSections.BACK_LEFT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_BR:
        error(errorSections.BACK_RIGHT, Color.kYellowGreen);
        break;

      case NO_ERROR:
      default:
        // Do Nothing
        break;
    }
  }

  private static void solid(LEDSection section, Color c1) {
    for (int i = section.startId(); i <= section.endId(); i++) {
      m_buffer.setLED(i, c1);
    }
  }

  private static void blink(LEDSection section, Color c1, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? c1 : Color.kBlack);
  }

  private static void flow(LEDSection section, Color c1, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio));
    double green = (c1.green * (1 - ratio));
    double blue = (c1.blue * (1 - ratio));
    solid(section, new Color(red, green, blue));
  }

  private static void breath(
      LEDSection section, Color c1, Color c2, double duration, double timestamp) {
    double x =
        ((timestamp % LEDConstants.breathDuration) / LEDConstants.breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  /**
   * Pulsing pattern of the LEDs
   *
   * @param c1 Color to be used
   * @param d1 Duration of the pulsing cycle (in seconds)
   * @param cycles Number of Pulses in each cycle
   */
  private static void pulse(LEDSection section, Color c1, double d1, double cycles) {
    boolean on = ((Timer.getFPGATimestamp() % (d1 * 2 * cycles)) / (d1 * 2 * cycles)) > 0.5;
    double red = 0, green = 0, blue = 0;

    if (on) {
      double x = (1 - ((Timer.getFPGATimestamp() % d1) / d1)) * Math.PI;
      // double ratio = (Math.sin(x) + 1.0);
      double ratio = 1 / x;

      red = (c1.red * (1 - ratio));
      green = (c1.green * (1 - ratio));
      blue = (c1.blue * (1 - ratio));
    }
    solid(section, new Color(red, green, blue));
  }

  private static void wave(
      LEDSection section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;

    for (int i = section.startId(); i <= section.endId(); i++) {
      x += xDiffPerLed;
      // if (i >= LEDConstants.kLEDLength){
      double ratio = (Math.pow(Math.sin(x), LEDConstants.waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), LEDConstants.waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      m_buffer.setLED(i, new Color(red, green, blue));
      // m_LEDs.setData(m_buffer);
    }
  }

  private static void rainbow(LEDSection section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.startId(); i <= section.endId(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      m_buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  @Override
  public void periodic() {

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        m_currentColor = new Color[] {Color.kRed, Color.kBlack};
      }
      if (ally.get() == Alliance.Blue) {
        m_currentColor = new Color[] {Color.kNavy, Color.kBlack};
      }
    } else {
      m_currentColor = new Color[] {Color.kHotPink, Color.kPink};
      setMode(LedMode.NOT_CONNECTED);
    }

    if (DriverStation.isEStopped()) {
      setErrorMode(LedErrorMode.ERROR_ESTOP);
    }

    updateMainLeds();
    updateAmbienceLeds();
    updateErrorLeds();

    m_LEDs.setData(m_buffer);
  }

  public Command setModeCommand(LedMode newMode) {
    return startEnd(() -> setMode(newMode), () -> setMode(LedMode.DEFAULT))
        .withName("[LEDManager] SetMode");
  }
}
