package frc.robot.subsystems.arm.extension;

import com.revrobotics.SparkMaxAlternateEncoder;

public class ExtConstants {
  public static final int extensionCANID_L = 20;

  // Using Through Bore Encoder
  public static final SparkMaxAlternateEncoder.Type kEncType =
      SparkMaxAlternateEncoder.Type.kQuadrature;
  public static final int kCPR = 8192;

  public static final double kMinExtInches = 0;
  public static final double kMaxExtInches = 12;
  public static final double kExtPulleyDiameter = 2; // Inches

  // public static final float kExtMinExtRotDeg = (float) 0; // Degres
  // public static final float kExtMaxExtRotDeg =
  //     (float) ((kMaxExtInches - kMinExtInches) / (Math.PI * kExtPulleyDiameter)) * 360; //
  // Degres

  public static final int kCurrentLimit = 30;
  public static final int kZeroingCurrent = 30;

  public static final double kExtensionEncoderPositionFactorDeg = 0.333;
  public static final double kExtensionEncoderVelocityFactorDeg = 0.333;

  public static final double kExtensionEncoderPositionPIDMinInput = 0; // inches
  public static final double kExtensionEncoderPositionPIDMaxInput =
      kExtensionEncoderPositionFactorDeg * kMaxExtInches; // inches

  public static final double kExtP = 0.3;
  public static final double kExtI = 0;
  public static final double kExtD = 0;
  public static final double kExtFF = 0.002;
  public static final double kExtMinOutput = -1;
  public static final double kExtMaxOutput = +1;

  public static final double kAtInchesTolerance = 0.5;
}
