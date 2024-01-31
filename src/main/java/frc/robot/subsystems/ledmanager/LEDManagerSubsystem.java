// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ledmanager;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDManagerSubsystem extends SubsystemBase {
  public static int kDIOCom1 = 0; // DIO
  public static int kDIOCom2 = 1; // DIO
  public static int kDIOCom3 = 2; // DIO

  public static byte kColorOFF          = 0b000;
  public static byte kColorCONE         = 0b001;
  public static byte kColorCUBE         = 0b010;
  public static byte kColorALLIANCEBLUE = 0b011;
  public static byte kColorALLIANCERED  = 0b100;
  public static byte kColorPINK         = 0b110;
  public static byte kColorWHITE        = 0b101;
  public static byte kColorRAINBOW      = 0b111;

  private DigitalOutput m_pin1 = new DigitalOutput(kDIOCom1);
  private DigitalOutput m_pin2 = new DigitalOutput(kDIOCom2);
  private DigitalOutput m_pin3 = new DigitalOutput(kDIOCom3);

  private byte m_pin1Mask = 0b00000001;
  private byte m_pin2Mask = 0b00000010;
  private byte m_pin3Mask = 0b00000100;

  /** Class Constructor. */
  public LEDManagerSubsystem() {
  }

  public void setColor(byte newColor){
    m_pin1.set(((newColor & m_pin1Mask) == m_pin1Mask)? true : false);
    m_pin2.set(((newColor & m_pin2Mask) == m_pin2Mask)? true : false);
    m_pin3.set(((newColor & m_pin3Mask) == m_pin3Mask)? true : false);
  }

  public void setAllianceColor() {
      // Set LEDS to Alliance Color
      if(DriverStation.getAlliance() == Optional.of(Alliance.Blue)){
        setColor(kColorALLIANCEBLUE); 
        // System.out.printf("[setAllianceColor] BLUE\n"); 

      } else {
        setColor(kColorALLIANCERED);
        // System.out.printf("[setAllianceColor] RED\n"); 

      }
  }
  public byte getAllianceColor() {
    // Set LEDS to Alliance Color
      if(DriverStation.getAlliance() == Optional.of(Alliance.Blue)){
      return kColorALLIANCEBLUE;
    } else {
      return kColorALLIANCERED;
    }
  }
}

