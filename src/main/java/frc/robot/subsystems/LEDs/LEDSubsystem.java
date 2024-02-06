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
    ESTOPPED,
    AUTO_FINISHED,
    AUTONOMOUS,
    HANGING,
    SHOOTING,
    CONTAINING,
    INTAKING,
    BLUE_ALLIANCE,
    RED_ALLIANCE,
    LOW_BATTERY_ALERT,
    DISABLED,
    OFF,
  }

  LedMode mode = LedMode.OFF;

  public void setMode(LedMode m_mode) {
    mode = m_mode;
  }

  public LEDSubsystem() {

    m_LEDs = new AddressableLED(LEDConstants.kLEDPort);
    m_buffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);
    m_LEDs.setLength(LEDConstants.kLEDLength);
    m_LEDs.setData(m_buffer);
    m_LEDs.start();
    // this.setColor(Color.kRed);

  }

  public void setColor(Color color) {
    int kLEDBufferLength = 30;
    // AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLEDBufferLength);
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
      m_buffer.setLED(i, color);
    }
    m_LEDs.setData(m_buffer);
  }

  private void updateLeds() {
    switch (mode) {
      case SHOOTING:
        this.wave(Color.kOrange, Color.kYellow, 7.0, 2.0);
        break;
      case BLUE_ALLIANCE:
        this.wave(Color.kBlue, Color.kNavy, 7.0, 2.0);
        break;

      default:
        this.wave(Color.kBlack, Color.kWhite, 7.0, 2.0);
        break;
    }
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

// Optional<Alliance> alliance = DriverStation.getAlliance();
//     if (alliance.isPresent()) {
//       if (alliance.get() == Alliance.Blue) {
//         this.wave(Color.kBlue, Color.kNavy, 7.0, 2.0);
//       }
//       if (alliance.get() == Alliance.Red) {
//         this.wave(Color.kRed, Color.kMaroon, 7.0, 2.0);
//       }
//     } else {
//       this.wave(Color.kPink, Color.kWhite, 7.0, 2.0);
//     }
//     }
//   }
