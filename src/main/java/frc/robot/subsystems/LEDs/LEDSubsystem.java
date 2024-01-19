// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private enum LEDStates{
  ORANGE, YELLOW, GREEN, PURPLE , PINK, RAINBOW
  }
  private LEDStates m_state;
  private static AddressableLED m_AddressableLED = new AddressableLED(LEDConstants.kLEDLength);
  public LEDSubsystem(){
    
  }
  public void setColor(){
     int kLEDBufferLength = 30;
     AddressableLEDBuffer buffer = new AddressableLEDBuffer(kLEDBufferLength);
    for (int i = 0; i < LEDConstants.kLEDLength; i++) {
      if(m_AddressableLED == BLUE){
        buffer.setRGB(i, 0, 0, 255);
      }
      if(m_AddressableLED == RED){
        buffer.setRGB(i, 255, 0, 0);
      }
    }
    m_AddressableLED.setData(buffer);

  }
    public void getColor(){
      
    }

  @Override
 
  
    public void periodic() {
    // This method will be called once per scheduler run
    while(AddressableLED.setColor() != LEDConstants.kErrorLight){
      return allianceColor;
    }
    return kErrorLight.getColor();
    
  }
}
