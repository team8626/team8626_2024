package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
  ;
}
