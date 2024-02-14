package frc.robot.subsystems.arm;

import com.revrobotics.SparkMaxAlternateEncoder;

public class ArmConstants {

  public static final class Extension {
<<<<<<< HEAD
<<<<<<< HEAD
    public static final int extensionCANID_L = 20;
=======
    public static final int extensionCANID_L = 30;
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
    public static final int extensionCANID_L = 20;
>>>>>>> 13d8b35 (Update Vendor Libraries to latest version)

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
<<<<<<< HEAD
    public static final int kZeroingCurrent = 25;

    public static final double kExtensionEncoderPositionFactorDeg = 360; // degres
    public static final double kExtensionEncoderVelocityFactorDeg = 360 / 60; // degres per second
=======
    public static final int kZeroingCurrent = 5; // TODO: Define the appropriate value!

    public static final double kExtensionEncoderVelocityFactorDeg = 360; // degres
    public static final double kExtensionEncoderPositionFactorDeg = 360 / 60; // degres per second
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))

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
    public static final int rotationCANID_L = 22;
    public static final int rotationCANID_R = 23;

    public static final float kMinRotDeg = -103;
    public static final float kMaxRotDeg = 20;

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
    public static final double kMaxAngleForSafeRetraction = 10;
  }

  public static class Preset {
    private double m_rot = 0;
    private double m_ext = 0;

    public Preset(double newRotDeg, double newExtInches) {
      m_rot = newRotDeg;
      m_ext = newExtInches;
    }

<<<<<<< HEAD
<<<<<<< HEAD
    public double getRotDegrees() {
=======
    public double getRotegrees() {
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
    public double getRotDegrees() {
>>>>>>> 0dbf64e (Autonomous Command Frames and Cleanup)
      return m_rot;
    }

    public double getExtInches() {
      return m_ext;
    }
  }

  public static final class Presets {
<<<<<<< HEAD
    public static final Preset kStart = new Preset(45, 0);
=======
    public static final Preset kStxart = new Preset(45, 0);
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
    public static final Preset kStow = new Preset(10, 0);
    public static final Preset kFloorPickup = new Preset(20, 5);
    public static final Preset kAmp = new Preset(-110, 10);
    public static final Preset kShootSpeaker_0ft = new Preset(15, 4);
    public static final Preset kShootSpeaker_3ft = new Preset(15, 4);
    public static final Preset kShootSpeaker_5ft = new Preset(15, 4);
    public static final Preset kStartClimb = new Preset(100, 0);
<<<<<<< HEAD
<<<<<<< HEAD
    public static final Preset kShootAmplifier_0ft = new Preset(15, 4);
    public static final Preset kShootAmplifier_3ft = new Preset(15, 4);
    public static final Preset kShootAmplifier_5ft = new Preset(15, 4);
=======
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
    public static final Preset kShootAmplifier_0ft = new Preset(15, 4);
    public static final Preset kShootAmplifier_3ft = new Preset(15, 4);
    public static final Preset kShootAmplifier_5ft = new Preset(15, 4);
>>>>>>> 0dbf64e (Autonomous Command Frames and Cleanup)
  }
  ;
}
