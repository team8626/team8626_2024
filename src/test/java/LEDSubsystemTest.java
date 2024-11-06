import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.AddressableLED;
import frc.robot.subsystems.LEDs.LEDConstants;
import frc.robot.subsystems.LEDs.LEDConstants.LedErrorMode;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LEDSubsystemTest {
  AddressableLED m_LEDs;
  LedMode m_mode;
  LedErrorMode m_error_mode;
  static final double DELTA = 1e-2; // acceptable deviation range

  @BeforeEach // this method will run before each test
  void setup() {
    m_LEDs = new AddressableLED(LEDConstants.kLEDPort); // create our LED Subsystem
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    m_LEDs.close(); // destroy our LED object
  }

  @Test
  void worksWhenOpen() {
    assertEquals(m_error_mode, LedErrorMode.NO_ERROR, DELTA);
  }
}
