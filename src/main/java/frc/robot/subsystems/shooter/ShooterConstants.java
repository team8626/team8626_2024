// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public class ShooterConstants {
  public static final int kCANMotor1 = 24;
  public static final int kCANMotor2 = 25;
  public static final int kMaxRPM = 5700;

  public static final int kRPMTolerance = 100;

  public class ShooterStates {
    public static enum ShooterStatus {
      IDLE(0, "idle"),
      RAMPUP(1, "Ramping Up"),
      ATSPEED(1, "At Speed");

      private final int m_id;
      private final String m_string;

      ShooterStatus(int newId, String newString) {
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
