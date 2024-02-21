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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDs.LEDConstants.LedErrorMode;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDConstants.errorSections;
import java.util.Optional;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private static AddressableLED m_LEDs;

  private static AddressableLEDBuffer m_buffer;

  static LedMode m_mode = LedMode.OFF;
  LedErrorMode m_error_mode = LedErrorMode.NO_ERROR;
  static Color m_currentColor[] = {Color.kHotPink, Color.kPink};

  public LEDSubsystem() {

    m_LEDs = new AddressableLED(LEDConstants.kLEDPort);
    m_buffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
    m_LEDs.setLength(LEDConstants.kLEDLength);
    m_LEDs.setData(m_buffer);
    m_LEDs.start();

    setMode(LedMode.NOT_CONNECTED);
    // setErrorMode(LedErrorMode.ERROR_CRITICAL);

    // m_buffer.setLED(10, Color.kRed);
    // m_error_mode = LedErrorMode.ERROR_CRITICAL;
  }

  public static void setMode(LedMode newMode) {
    m_mode = newMode;
    updateLeds();
  }

  public void setErrorMode(LedErrorMode newErrorCode) {
    m_error_mode = newErrorCode;
    m_LEDs.setData(m_buffer);
    updateErrorLeds();
  }

  /**
   * Set color of the complete LED Buffer to color
   *
   * @param color
   */
  public void setColor(Color color) {
    // AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLEDBufferLength);
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
      m_buffer.setLED(i, color);
    }
    // m_LEDs.setData(m_buffer);
  }

  /**
   * Set color of the all Error LEDs
   *
   * @param color
   */
  public void error(Color color) {
    boolean on = ((Timer.getFPGATimestamp() % 2) / 2) > 0.5;
    for (int i = 0; i < LEDConstants.kErrorLightIndex.length; i++) {
      for (int j = 0; j < LEDConstants.kErrorLightIndex[i].length; j++) {
        int ledIndex = LEDConstants.kErrorLightIndex[i][j];
        if (on) m_buffer.setLED(ledIndex, color);
        else m_buffer.setLED(ledIndex, Color.kBlack);
        // m_LEDs.setData(m_buffer);
      }
    }
  }

  /**
   * Set color of the specified Error Section
   *
   * @param section
   * @param color
   */
  public void error(errorSections section, Color color) {
    boolean on = ((Timer.getFPGATimestamp() % 2) / 2) > 0.5;

    if (section.getValue() < LEDConstants.kErrorLightIndex.length) {
      for (int j = 0; j < LEDConstants.kErrorLightIndex[section.getValue()].length; j++) {
        int ledIndex = LEDConstants.kErrorLightIndex[section.getValue()][j];
        if (on) m_buffer.setLED(ledIndex, color);
        else m_buffer.setLED(ledIndex, Color.kBlack);
        // m_LEDs.setData(m_buffer);
      }
    }
  }

  /**
   * Set color of the specified Error Sections
   *
   * @param sections
   * @param color
   */
  public void error(errorSections[] sections, Color color) {
    boolean on = ((Timer.getFPGATimestamp() % 2) / 2) > 0.5;

    for (int i = 0; i < sections.length; i++) {
      int section = sections[i].getValue();
      if (section < LEDConstants.kErrorLightIndex.length) {
        for (int j = 0; j < LEDConstants.kErrorLightIndex[section].length; j++) {
          int ledIndex = LEDConstants.kErrorLightIndex[section][j];
          if (on) m_buffer.setLED(ledIndex, color);
          else m_buffer.setLED(ledIndex, Color.kBlack);
          // m_LEDs.setData(m_buffer);
        }
      }
    }
  }

  /** Update LEDs based on current set state */
  private static void updateLeds() {
    switch (m_mode) {
      case NOT_CONNECTED:
        // wave(m_currentColor[0], m_currentColor[1], 25, 2.0);
        // blink(Color.kLightPink, 0.5);
        // flow(m_currentColor[0], 2.0);
        pulse(m_currentColor[0], 0.5, 2.0);
        break;
      case DISABLED:
        breath(
            m_currentColor[0],
            m_currentColor[1],
            LEDConstants.breathDuration,
            Timer.getFPGATimestamp());
        break;
      case INTAKING:
      case SHOOTING:
        flow(m_currentColor[0], 0.5);
        break;
      case TEST:
        rainbow(LEDConstants.waveSlowCycleLength, LEDConstants.waveSlowDuration);
        break;

      case FOLLOWNOTE:
        flow(Color.kOrange, 1);
        break;

      case DEFAULT:
      default:
        wave(m_currentColor[0], m_currentColor[1], 25, 2.0);
        break;
    }
  }

  /** Update ErrorLEDs based on current set state */
  private void updateErrorLeds() {

    switch (m_error_mode) {
      case ERROR_CRITICAL:
        this.error(Color.kYellowGreen);
        break;
      case ERROR_DRIVE_FL:
        this.error(errorSections.FRONT_LEFT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_FR:
        this.error(errorSections.FRONT_RIGHT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_BL:
        this.error(errorSections.BACK_LEFT, Color.kYellowGreen);
        break;
      case ERROR_DRIVE_BR:
        this.error(errorSections.BACK_RIGHT, Color.kYellowGreen);
        break;

      default:
        // Do Nothing
        break;
    }
  }

  private static void solid(Color c1) {
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
      m_buffer.setLED(i, c1);
      // m_LEDs.setData(m_buffer);
    }
  }

  private static void blink(Color c1, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(on ? c1 : Color.kBlack);
  }

  private static void flow(Color c1, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio));
    double green = (c1.green * (1 - ratio));
    double blue = (c1.blue * (1 - ratio));
    solid(new Color(red, green, blue));
  }

  private static void breath(Color c1, Color c2, double duration, double timestamp) {
    double x =
        ((timestamp % LEDConstants.breathDuration) / LEDConstants.breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

  /**
   * Pulsing pattern of the LEDs
   *
   * @param c1 Color to be used
   * @param d1 Duration of the pulsing cycle (in seconds)
   * @param cycles Number of Pulses in each cycle
   */
  private static void pulse(Color c1, double d1, double cycles) {
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
    solid(new Color(red, green, blue));
  }

  private static void wave(Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
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
      // }
    }
  }

  private static void rainbow(double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
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
        m_currentColor = new Color[] {Color.kRed, Color.kDarkRed};
      }
      if (ally.get() == Alliance.Blue) {
        m_currentColor = new Color[] {Color.kBlue, Color.kAqua};
      }
    } else {
      m_currentColor = new Color[] {Color.kHotPink, Color.kPink};
      setMode(LedMode.NOT_CONNECTED);
    }

    // if (!DriverStation.isDSAttached()) {
    //   m_currentColor = new Color[] {Color.kHotPink, Color.kPink};
    //   setMode(LedMode.NOT_CONNECTED);
    // } else if (!DriverStation.isFMSAttached()) {
    //   m_currentColor = new Color[] {Color.kHotPink, Color.kPink};
    //   setMode(LedMode.NOT_CONNECTED);
    // }

    if (DriverStation.isEStopped()) {
      setErrorMode(LedErrorMode.ERROR_ESTOP);
    }

    updateLeds();
    updateErrorLeds();

    m_LEDs.setData(m_buffer);
  }
}
