package frc.robot.subsystems.arm.rotation;

public class RotConstants {
  public static final int rotationCANID_L = 22;
  public static final int rotationCANID_R = 23;

  public static final float kMinRotDeg = 77;
  public static final float kMaxRotDeg = 205;

  public static final int kCurrentLimit = 30;

  public static final double kRotationEncoderPositionFactorDeg = 360; // degres
  public static final double kRotationEncoderVelocityFactorDeg = 360 / 60; // degres per second

  public static final double kRotationEncoderPositionPIDMinInput = 0; // degrees
  public static final double kRotationEncoderPositionPIDMaxInput =
      kRotationEncoderPositionFactorDeg; // degrees

  public static final double kRotP = 0.06;
  public static final double kRotI = 0.00;
  public static final double kRotD = 0.00;
  public static final double kRotFF = 0.00;
  public static final double kRotMinOutput = -1;
  public static final double kRotMaxOutput = +0.1;

  public static final double kMaxAngleForSafeRetraction = 195;

  public static final double kAtAngleTolerance = 1;
}
