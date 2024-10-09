// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ShooterConstants {
  public static final int kCANMotor1 = 24;
  public static final int kCANMotor2 = 25;
  public static final int kShooterGearRatio = 2;
  public static final int kMaxRPM = 5500 * kShooterGearRatio; // Max NEO Speed * Gear Ratio;
  public static final double kFlywheelDiameterMeters = Units.inchesToMeters(3);

  public static final double kP_top = 0.00055;
  public static final double kI_top = 0.000;
  public static final double kD_top = 0.000;
  public static final double kIz_top = 2500;
  public static final double kFF_top = 0.00020;
  public static final double kMaxOutput_top = 1;
  public static final double kMinOutput_top = -1;

  public static final double kP_bottom = 0.00055;
  public static final double kI_bottom = 0.000;
  public static final double kD_bottom = 0.000;
  public static final double kIz_bottom = 2500;
  public static final double kFF_bottom = 0.00020;
  public static final double kMaxOutput_bottom = 1;
  public static final double kMinOutput_bottom = -1;

  public static final int kRPMTolerance = 200;

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
