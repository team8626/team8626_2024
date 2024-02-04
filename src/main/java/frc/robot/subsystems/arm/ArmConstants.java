package frc.robot.subsystems.arm;

public class ArmConstants {

  public static final class Extension {
    public static final int extensionCANID_L = 30;
    // public static final int extensionCANID_R = 31; // Unused, Only 1 motor

    public static final float kExtensionMinDeg = -360;
    public static final float kExtensionMaxDeg = 360;

    public static final float kExtensionMinLengthInches = 0;
    public static final float kExtensionMaxLengthInches = 10;

    public static final int kCurrentLimit = 30;

    public static final double kExtensionEncoderVelocityFactorDeg = 360; // degres
    public static final double kExtensionEncoderPositionFactorDeg = 360 / 60; // degres per second

    public static final double kExtensionEncoderPositionPIDMinInput = 0; // degrees
    public static final double kExtensionEncoderPositionPIDMaxInput =
        kExtensionEncoderPositionFactorDeg; // degrees

    public static final double kExtP = 0.02;
    public static final double kExtI = 0;
    public static final double kExtD = 0;
    public static final double kExtFF = 0;
    public static final double kExtMinOutput = -0.5;
    public static final double kExtMaxOutput = +0.5;
  }

  public static final class Rotation {
    public static final int rotationCANID_L = 32;
    public static final int rotationCANID_R = 33;

    public static final float kRotationMinDeg = -90;
    public static final float kRotationMaxDeg = 90;

    public static final int kCurrentLimit = 30;

    public static final double kRotationEncoderVelocityFactorDeg = 360; // degres
    public static final double kRotationEncoderPositionFactorDeg = 360 / 60; // degres per second

    public static final double kRotationEncoderPositionPIDMinInput = 0; // degrees
    public static final double kRotationEncoderPositionPIDMaxInput =
        kRotationEncoderVelocityFactorDeg; // degrees

    public static final double kRotP = 0.02;
    public static final double kRotI = 0;
    public static final double kRotD = 0;
    public static final double kRotFF = 0;
    public static final double kRotMinOutput = -0.5;
    public static final double kRotMaxOutput = +0.5;

    public static final double kReelDiameterInches = 2;
  }

  public static final class Presets {
    public static final int kStart[] = {45, 0};
    public static final int kStow[] = {45, 0};
    public static final int kFloorPickup[] = {45, 0};
    public static final int kAmp[] = {45, 0};
    public static final int kShootSpeaker_0ft[] = {45, 0};
    public static final int kShootSpeaker_3ft[] = {45, 0};
    public static final int kShootSpeaker_5ft[] = {45, 0};
    public static final int kShoStartClimb[] = {45, 0};
  }
}
