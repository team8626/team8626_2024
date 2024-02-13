// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private static AddressableLED m_LEDs;

  private AddressableLEDBuffer m_buffer;

  public enum LedMode {
    NOT_CONNECTED,
    CONNECTED,
    ASTOPPED,
    ESTOPPED,
    AUTO_FINISHED,
    AUTONOMOUS,
    CLIMBING,
    HANGING,
    SHOOTING,
    CONTAINING,
    INTAKING,
    BLUE_ALLIANCE,
    RED_ALLIANCE,
    LOW_BATTERY_ALERT,
    DISABLED,
    OFF
  }

  public enum LedErrorMode {
    ERROR,
    ERROR_ASTOP,
    ERROR_ESTOP,
    ERROR_CRITICAL,
    NO_ERROR,
    ERROR_AUTONOMOUS,
    ERROR_CLIMBING,
    ERROR_HANGING,
    ERROR_INTAKING,
    ERROR_CONTAINING,
    ERROR_SHOOTING,
    ERROR_ALLIANCE_SELECT,
  }

  public enum errorSections {
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3),
    MIDDLE_TOP(4);

    private final int id;

    errorSections(int id) {
      this.id = id;
    }

    public int getValue() {
      return id;
    }
  }

  LedMode m_mode = LedMode.OFF;
  LedErrorMode m_error_mode = LedErrorMode.NO_ERROR;

  public void setMode(LedMode newMode) {
    m_mode = newMode;
  }

  public void setErrorMode(LedErrorMode newErrorCode) {
    m_error_mode = newErrorCode;
  }

  public LEDSubsystem() {

    m_LEDs = new AddressableLED(LEDConstants.kLEDPort);
    m_buffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
    m_LEDs.setLength(LEDConstants.kLEDLength);
    m_LEDs.setData(m_buffer);
    m_LEDs.start();
    // this.setColor(Color.kRed);

    m_mode = LedMode.BLUE_ALLIANCE;
    m_error_mode = LedErrorMode.ERROR_CRITICAL;
  }

  public void setColor(Color color) {
    // AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLEDBufferLength);
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
      m_buffer.setLED(i, color);
    }
    m_LEDs.setData(m_buffer);
  }

  public void error(Color color) {
    boolean on = ((Timer.getFPGATimestamp() % 2) / 2) > 0.5;
    for (int i = 0; i < LEDConstants.kErrorLightIndex.length; i++) {
      for (int j = 0; j < LEDConstants.kErrorLightIndex[i].length; j++) {
        int ledIndex = LEDConstants.kErrorLightIndex[i][j];
        if (on) m_buffer.setLED(ledIndex, color);
        else m_buffer.setLED(ledIndex, Color.kBlack);
        m_LEDs.setData(m_buffer);
      }
    }
  }

  public void error(errorSections section, Color color) {
    boolean on = ((Timer.getFPGATimestamp() % 2) / 2) > 0.5;

    if (section.getValue() < LEDConstants.kErrorLightIndex.length) {
      for (int j = 0; j < LEDConstants.kErrorLightIndex[section.getValue()].length; j++) {
        int ledIndex = LEDConstants.kErrorLightIndex[section.getValue()][j];
        if (on) m_buffer.setLED(ledIndex, color);
        else m_buffer.setLED(ledIndex, Color.kBlack);
        m_LEDs.setData(m_buffer);
      }
    }
  }

  public void error(errorSections[] sections, Color color) {
    boolean on = ((Timer.getFPGATimestamp() % 2) / 2) > 0.5;

    for (int i = 0; i < sections.length; i++) {
      int section = sections[i].getValue();
      if (section < LEDConstants.kErrorLightIndex.length) {
        for (int j = 0; j < LEDConstants.kErrorLightIndex[section].length; j++) {
          int ledIndex = LEDConstants.kErrorLightIndex[section][j];
          if (on) m_buffer.setLED(ledIndex, color);
          else m_buffer.setLED(ledIndex, Color.kBlack);
          m_LEDs.setData(m_buffer);
        }
      }
    }
  }

  private void updateLeds() {
    switch (m_mode) {
      case ESTOPPED:
        this.blink(Color.kHotPink, 2.0);
      case ASTOPPED:
        this.blink(Color.kCrimson, 2.0);
      case NOT_CONNECTED:
        this.wave(Color.kYellow, Color.kGold, 7.0, 2.0);
        break;
      case CONNECTED:
        this.flow(Color.kPink, 0, 2.0);
        break;
      case AUTONOMOUS:
        this.solid(Color.kSkyBlue);
        break;
      case INTAKING:
        this.wave(Color.kGreen, Color.kLime, 7.0, 2.0);
        break;
      case CONTAINING:
        this.wave(Color.kGreen, Color.kLime, 7.0, 2.0);
        break;
      case SHOOTING:
        this.wave(Color.kOrange, Color.kOrangeRed, 7.0, 2.0);
        break;
      case CLIMBING:
        this.flow(Color.kDarkOrchid, 0, 1.0);
      case HANGING:
        this.solid(Color.kPink);
      case BLUE_ALLIANCE:
        this.wave(Color.kBlue, Color.kNavy, 7.0, 2.0);
        break;
      case RED_ALLIANCE:
        this.wave(Color.kRed, Color.kMaroon, 7.0, 2.0);
        break;
      case DISABLED:
        this.solid(Color.kGray);
        break;
      default:
        this.error(Color.kDarkSalmon);
        break;
    }

    switch (m_error_mode) {
      case ERROR:
        this.error(Color.kDarkMagenta);
      case ERROR_CRITICAL:
        this.error(errorSections.MIDDLE_TOP, Color.kOrange);
        this.error(
            new errorSections[] {errorSections.FRONT_RIGHT, errorSections.FRONT_LEFT},
            Color.kOrange);

      default:
        // Do Nothing
        break;
    }
  }

  private void solid(Color c1) {
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
      m_buffer.setLED(i, c1);
      m_LEDs.setData(m_buffer);
    }
  }

  private void blink(Color c1, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(on ? c1 : Color.kBlack);
  }

  private void flow(Color c1, int start, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio));
    double green = (c1.green * (1 - ratio));
    double blue = (c1.blue * (1 - ratio));
    solid(new Color(red, green, blue));
  }

  private void wave(Color c1, Color c2, double cycleLength, double duration) {
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
      m_LEDs.setData(m_buffer);
      // }
    }
  }

  @Override
  public void periodic() {

    if (DriverStation.isEStopped()) {
      setErrorMode(LedMode.ERROR_ESTOP);
    } else if (DriverStation.isAStopped()) {
      setErrorMode(LedMode.ERROR_ASTOP);
    }

    updateLeds();

    // This method will be called once per scheduler run

    // if (DriverStation.isDSAttached()) {
    // if (DriverStation.getAlliance().get() == Alliance.Blue) {
    // this.wave(Color.kAquamarine, Color.kAzure, 25.0, 2.0);
    // }
    // }

    // Check DS Status
    // -> Not Attached => Fushia/Pink wave(P1, P2, x, y)
    // -> Attached => Alliance Color Blue? B1, B2... Red? R1, R2...

  }
}
