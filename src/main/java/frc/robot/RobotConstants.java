package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class RobotConstants {

  public static Translation2d intakeOffset =
      new Translation2d(Units.inchesToMeters(24), Units.inchesToMeters(0));

  public static double kFrameWidth = Units.inchesToMeters(24);
  public static double kFrameLength = Units.inchesToMeters(24);

  public static double kArmOffsetXMeters = Units.inchesToMeters(-6.50);
  public static double kArmOffsetYMeters = Units.inchesToMeters(0);
  public static double kArmOffsetZMeters = Units.inchesToMeters(16.5);
  public static double kArmRotationRollOffsetDeg = 0;
  public static double kArmRotationPitchOffsetDeg = -90;
  public static double kArmRotationYawOffsetDeg = 0;
  public static double kArmExtensionOffMeters = Units.inchesToMeters(-3);

  public static double kAutoSpinRadius = 2;

  public static class Vision {
    // april tag camera constants
    public static final String kATCameraName = "Arducam_AT002";
    public static final Transform3d kATRobotToCam =
        new Transform3d(
            new Translation3d(-0.222275, -0.275290, 0.249531),
            new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(167.04746)));

    // Object Detection camera Constants
    public static final String kODCameraName = "Arducam_OD003";
    public static final Pose3d kODCamPose =
        new Pose3d(
            new Translation3d(0.208264, 0, 0.346066),
            new Rotation3d(0, Units.degreesToRadians(20), 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }
}
