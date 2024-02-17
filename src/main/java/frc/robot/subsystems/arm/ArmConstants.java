package frc.robot.subsystems.arm;

import com.revrobotics.SparkMaxAlternateEncoder;

public class ArmConstants {

  public static final class Extension {
    public static final int extensionCANID_L = 20;

    // Using Through Bore Encoder
    public static final SparkMaxAlternateEncoder.Type kAltEncType =
        SparkMaxAlternateEncoder.Type.kQuadrature;
    public static final int kCPR = 8192;

    public static final double kMinExtInches = 0;
    public static final double kMaxExtInches = 10;
    public static final double kExtPulleyDiameter = 2; // Inches

    public static final float kExtMinExtRotDeg = (float) 0; // Degres
    public static final float kExtMaxExtRotDeg =
        (float) ((kMaxExtInches - kMinExtInches) / (Math.PI * kExtPulleyDiameter)) * 360; // Degres

    public static final int kCurrentLimit = 30;
    public static final int kZeroingCurrent = 25;

    public static final double kExtensionEncoderPositionFactorDeg = 360; // degres
    public static final double kExtensionEncoderVelocityFactorDeg = 360 / 60; // degres per second

    public static final double kExtensionEncoderPositionPIDMinInput = 0; // degrees
    public static final double kExtensionEncoderPositionPIDMaxInput =
        kExtensionEncoderPositionFactorDeg; // degrees

    public static final double kExtP = 0.02;
    public static final double kExtI = 0;
    public static final double kExtD = 0;
    public static final double kExtFF = 0;
    public static final double kExtMinOutput = -0.1;
    public static final double kExtMaxOutput = +0.1;
  }

  public static final class Rotation {
    public static final int rotationCANID_L = 22;
    public static final int rotationCANID_R = 23;

    public static final float kMinRotDeg = 120;
    public static final float kMaxRotDeg = 205;

    public static final int kCurrentLimit = 10; // TODO; was 30

    public static final double kRotationEncoderPositionFactorDeg = 360; // degres
    public static final double kRotationEncoderVelocityFactorDeg = 360 / 60; // degres per second

    public static final double kRotationEncoderPositionPIDMinInput = 0; // degrees
    public static final double kRotationEncoderPositionPIDMaxInput =
        kRotationEncoderPositionFactorDeg; // degrees

    public static final double kRotP = 0.02;
    public static final double kRotI = 0;
    public static final double kRotD = 0;
    public static final double kRotFF = 0;
    public static final double kRotMinOutput = -1;
    public static final double kRotMaxOutput = +1;

    public static final double kReelDiameterInches = 2;
    public static final double kMaxAngleForSafeRetraction = 10;
  }
}
