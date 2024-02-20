// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class IntakeConstants {
  public static final int kCANMotor1 = 26;
  public static final int kCANMotor2 = 27;
  public static final int kIRSensor1 = 0; // DIO
  public static final int kIRSensor2 = 1; // DIO

  public static final double kSpeed_Intake = 0.9;
  public static final double kSpeed_Coast = 0.4;
  public static final double kSpeed_Shoot = 1;
  public static final double kSpeed_Adjust = -0.1;
  public static final double kSpeed_Discard = -0.5;

  public class IntakeStates {
    public static enum IntakeStatus {
      IDLE(0, "idle"),
      INTAKING(1, "Intaking"),
      COASTING(2, "Coasting"),
      ADJUSTING(3, "Adjusting"),
      SHOOTING(4, "Shooting");

      private final int m_id;
      private final String m_string;

      IntakeStatus(int newId, String newString) {
        this.m_id = newId;
        this.m_string = newString.toUpperCase();
      }

      public String getString() {
        return m_string;
      }

      public double getId() {
        return m_id;
      }
    }
  }
}
