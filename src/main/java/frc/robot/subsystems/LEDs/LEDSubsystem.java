// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private Timer timer = new Timer();
  private static AddressableLED m_AddressableLED = new AddressableLED(LEDConstants.kLEDLength);
  
  public LEDSubsystem(){
    
  }

  public void setColor(Color color){
     int kLEDBufferLength = 30;
     AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLEDBufferLength);
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
      buffer.setLED(i, color);
    }
    m_AddressableLED.setData(buffer);

  }
    public void getColor(){
      
    }

  @Override
 
  
    public void periodic() {
    // This method will be called once per scheduler run

    
  }
}
