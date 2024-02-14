// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase implements ImplementDashboard {
  /** Create Motors * */
  private final CANSparkMax m_extensionMotor_L;
  // private final CANSparkMax m_extensionMotor_R;
  private final CANSparkMax m_rotationMotor_L;
  private final CANSparkMax m_rotationMotor_R;

  /** Create Sensors * */
  private final RelativeEncoder m_extensionEncoder;

  private final AbsoluteEncoder m_rotationEncoder;
  private final SparkPIDController m_extensionPIDController;
  private final SparkPIDController m_rotationPIDController;

  /** Class States * */
  private double m_desiredAngleDeg = 0;

  private double m_desiredExtensionInches = 0;

  private double m_currentAngleDeg = 0;
  private double m_currentExtInches = 0;

  private boolean m_armIsResetting = false;
  private boolean m_armZeroed = false;

<<<<<<< HEAD
<<<<<<< HEAD
  private static double atAngleTolerance = 0;
  private static double atInchesTolerance = 0;

=======
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
  private static double atAngleTolerance = 0;
  private static double atInchesTolerance = 0;

>>>>>>> a2e5544 (Set Arm Command)
  /** Poses publisher for AdvantageScope * */
  StructArrayPublisher<Pose3d> poseArrayPublisher =
      NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

  /** Creates a new ArmExtensionSubsystem. */
  public ArmSubsystem() {
    /** Motors * */
    m_extensionMotor_L =
        new CANSparkMax(ArmConstants.Extension.extensionCANID_L, MotorType.kBrushless);
    m_rotationMotor_L =
        new CANSparkMax(ArmConstants.Rotation.rotationCANID_L, MotorType.kBrushless);
    m_rotationMotor_R =
        new CANSparkMax(ArmConstants.Rotation.rotationCANID_R, MotorType.kBrushless);

    /** Reset & Initialize Controllers * */
    m_extensionMotor_L.restoreFactoryDefaults();

    m_rotationMotor_L.restoreFactoryDefaults();
    m_rotationMotor_R.restoreFactoryDefaults();
    m_rotationMotor_R.follow(m_rotationMotor_L, true);

    // Setup encoders and PID controllers for SPARKMAX.
    m_extensionEncoder =
        m_extensionMotor_L.getAlternateEncoder(
            ArmConstants.Extension.kAltEncType, ArmConstants.Extension.kCPR);

    m_extensionPIDController = m_extensionMotor_L.getPIDController();
    m_extensionPIDController.setFeedbackDevice(m_extensionEncoder);

    // Extension Soft Limits
    m_extensionMotor_L.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_extensionMotor_L.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_extensionMotor_L.setSoftLimit(
        SoftLimitDirection.kReverse, ArmConstants.Extension.kExtMinExtRotDeg);
    m_extensionMotor_L.setSoftLimit(
        SoftLimitDirection.kForward, ArmConstants.Extension.kExtMaxExtRotDeg);

    m_rotationEncoder = m_rotationMotor_L.getAbsoluteEncoder(Type.kDutyCycle);
    m_rotationPIDController = m_rotationMotor_L.getPIDController();
    m_rotationPIDController.setFeedbackDevice(m_rotationEncoder);

    // Rotation Soft Limits
    m_rotationMotor_L.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_rotationMotor_L.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_rotationMotor_L.setSoftLimit(SoftLimitDirection.kReverse, ArmConstants.Rotation.kMinRotDeg);
    m_rotationMotor_L.setSoftLimit(SoftLimitDirection.kForward, ArmConstants.Rotation.kMaxRotDeg);

    // Apply position and velocity conversion factors for the turning encoder.
    // We want these in degrees and degrees per second
    m_extensionEncoder.setPositionConversionFactor(
        ArmConstants.Extension.kExtensionEncoderPositionFactorDeg);
    m_extensionEncoder.setVelocityConversionFactor(
        ArmConstants.Extension.kExtensionEncoderVelocityFactorDeg);
    m_extensionMotor_L.setInverted(false);

    m_rotationEncoder.setPositionConversionFactor(
        ArmConstants.Rotation.kRotationEncoderPositionFactorDeg);
    m_rotationEncoder.setVelocityConversionFactor(
        ArmConstants.Rotation.kRotationEncoderVelocityFactorDeg);
<<<<<<< HEAD
    m_rotationMotor_L.setInverted(true);
=======
    m_rotationMotor_L.setInverted(false);
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))

    // Set the PID gains for the motor.
    m_extensionPIDController.setP(ArmConstants.Extension.kExtP);
    m_extensionPIDController.setI(ArmConstants.Extension.kExtI);
    m_extensionPIDController.setD(ArmConstants.Extension.kExtD);
    m_extensionPIDController.setFF(ArmConstants.Extension.kExtFF);
    m_extensionPIDController.setOutputRange(
        ArmConstants.Extension.kExtMinOutput, ArmConstants.Extension.kExtMaxOutput);
    m_extensionPIDController.setPositionPIDWrappingEnabled(false);

    m_rotationPIDController.setP(ArmConstants.Rotation.kRotP);
    m_rotationPIDController.setI(ArmConstants.Rotation.kRotI);
    m_rotationPIDController.setD(ArmConstants.Rotation.kRotD);
    m_rotationPIDController.setFF(ArmConstants.Rotation.kRotFF);
    m_rotationPIDController.setOutputRange(
        ArmConstants.Rotation.kRotMinOutput, ArmConstants.Rotation.kRotMaxOutput);
    m_rotationPIDController.setPositionPIDWrappingEnabled(false);

    // Idle Modes and max current
    m_extensionMotor_L.setIdleMode(IdleMode.kBrake);
    m_extensionMotor_L.setSmartCurrentLimit(ArmConstants.Extension.kCurrentLimit);

    m_rotationMotor_L.setIdleMode(IdleMode.kBrake);
    m_rotationMotor_L.setSmartCurrentLimit(ArmConstants.Rotation.kCurrentLimit);

<<<<<<< HEAD
<<<<<<< HEAD
    setAngleTolerance(3);
    setExtensionTolerance(1);

=======
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
    setAngleTolerance(3);
    setExtensionTolerance(1);

>>>>>>> a2e5544 (Set Arm Command)
    /** Initialize the Subsystem * */
    if (RobotBase.isReal()) {
      m_desiredAngleDeg = m_rotationEncoder.getPosition();
      m_desiredExtensionInches = getExtensionInchesFromDeg(m_extensionEncoder.getPosition());

      // TODO: Launch zeroing of the arm
      // this.reset();
    }
  }

  /**
   * Set Arm Length to a specific extension in Inches. The desired length will be forced between
   *
   * @param newLengthInches requested length kMinExtInches and kMaxExtInches
   */
  public void setLengthInches(double newLengthInches) {
    if (m_armZeroed) {
      m_desiredExtensionInches = newLengthInches; // Value Clamped in periodic()
    } else {
      System.out.println("[ARM] Arm not initialized, cannot set length");
    }
  }

  /**
   * Set Arm Rotation to a specific angle in Degres. The desired length will be forced between
   *
   * @param newLengthInches requested length kMinRotDeg and kMaxRotDeg
   */
  public void setAngleDeg(double newAngleDeg) {
    m_desiredAngleDeg = newAngleDeg; // Value Clamped in periodic()
  }

  private double getLengthInches() {
    return m_currentExtInches;
  }

  private double getAngleDeg() {
    return m_currentAngleDeg;
  }

  public void extend(double newSpeed) {
    if (Robot.isReal()) {
      if ((m_currentExtInches < ArmConstants.Extension.kMaxExtInches)
          && (m_currentExtInches > ArmConstants.Extension.kMinExtInches)) {
        m_extensionPIDController.setReference(newSpeed * 0.50, ControlType.kDutyCycle);
      }
    } else { // Simulation
      if (newSpeed > 0) {
        m_desiredExtensionInches += 0.5;
      } else if (newSpeed < 0) {
        m_desiredExtensionInches -= 0.5;
      }
    }
  }

  public void rotate(double newSpeed) {
    if (Robot.isReal()) {
      if ((m_currentAngleDeg < ArmConstants.Rotation.kMaxRotDeg)
          && (m_currentAngleDeg > ArmConstants.Rotation.kMinRotDeg)) {
        m_rotationPIDController.setReference(newSpeed * 0.50, ControlType.kDutyCycle);
      }
    } else { // Simulation
      if (newSpeed > 0) {
        m_desiredAngleDeg += 1;
      } else if (newSpeed < 0) {
        m_desiredAngleDeg -= 1;
      }
    }
  }

  public void reset() {
    // Move arm to safe angle for retracting
    if (m_currentAngleDeg > ArmConstants.Rotation.kMaxAngleForSafeRetraction) {
      this.setAngleDeg(ArmConstants.Rotation.kMaxAngleForSafeRetraction);
    }
    m_armZeroed = false;
    m_armIsResetting = true;
  }

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> a2e5544 (Set Arm Command)
  public void setAngleTolerance(double positionTolerance) {
    atAngleTolerance = positionTolerance;
  }

  public void setExtensionTolerance(double positionTolerance) {
    atInchesTolerance = positionTolerance;
  }

  public boolean atExtensionSetpoint() {
    return MathUtil.isNear(m_desiredExtensionInches, m_currentExtInches, atInchesTolerance);
  }

  public boolean atAngleSetpoint() {
    return MathUtil.isNear(m_desiredAngleDeg, m_currentAngleDeg, atAngleTolerance);
  }
<<<<<<< HEAD
=======
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
>>>>>>> a2e5544 (Set Arm Command)
  /*
   * Convert Reel Rotations from Extension Length
   */
  private double getDegreesFromExtensionInches(double newExtensionInches) {

    return Math.toDegrees(
        newExtensionInches / (Math.PI * ArmConstants.Rotation.kReelDiameterInches));
  }

  /*
   * Convert Extension Lemngth from Reel Rotations
   */
  private double getExtensionInchesFromDeg(double newExtensionRotationDegrees) {

    return Math.toRadians(newExtensionRotationDegrees) * ArmConstants.Rotation.kReelDiameterInches;
  }

  @Override
  public void periodic() {
    /** Check for value in Range * */
    m_desiredAngleDeg =
        MathUtil.clamp(
            m_desiredAngleDeg, ArmConstants.Rotation.kMinRotDeg, ArmConstants.Rotation.kMaxRotDeg);
    m_desiredExtensionInches =
        MathUtil.clamp(
            m_desiredExtensionInches,
            ArmConstants.Extension.kMinExtInches,
            ArmConstants.Extension.kMaxExtInches);

    /** Update Current Arm Positions */
    if (RobotBase.isReal()) {
      m_currentAngleDeg = m_rotationEncoder.getPosition();
      m_currentExtInches = this.getExtensionInchesFromDeg(m_extensionEncoder.getPosition());
    } else {
      updateSimValues();
    }

    /** Set Target Positions into the Controllers * */
    if (!m_armIsResetting) {
      // TODO: Is this Angle allowed based on current extension?
      m_rotationPIDController.setReference(m_desiredAngleDeg, ControlType.kPosition);

      // TODO: Is this Extension allowed based on current rotation?
      if (m_armZeroed) {
        m_extensionPIDController.setReference(
            this.getDegreesFromExtensionInches(m_desiredExtensionInches), ControlType.kPosition);
      } else {
        // System.out.println("[ARM] Arm not initialized, will not setReference");
      }
    }
    /** Resetting the Arm * */
    else {
      // Wait for safe angle before retracting
      if (MathUtil.isNear(
          ArmConstants.Rotation.kMaxAngleForSafeRetraction, m_currentAngleDeg, 3 /*degrees*/)) {
        /** Use Motor output current to detect low limit * */
        if (m_extensionMotor_L.getOutputCurrent() < ArmConstants.Extension.kZeroingCurrent) {
          // Resetting the Arm - Move backwards until Current raises for that motor
          m_extensionPIDController.setReference(-0.2, ControlType.kDutyCycle);
        }
        /** Reached current threshold, we found the zero!* */
        else {
          // Reset done!
          m_extensionPIDController.setReference(0, ControlType.kDutyCycle);
          m_currentExtInches = 0;
          m_desiredExtensionInches = 0;
          m_armZeroed = true;
          m_armIsResetting = false;
        }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 70353eb (Dashboard pre work)
      }
      /** Running simulation, Reset "done" * */
      if (Robot.isSimulation()) {
        m_currentExtInches = 0;
        m_desiredExtensionInches = 0;
        m_armZeroed = true;
        m_armIsResetting = false;
<<<<<<< HEAD
=======

        /** Running simulation, Reset "done" * */
        if (Robot.isSimulation()) {
          m_currentExtInches = 0;
          m_desiredExtensionInches = 0;
          m_armZeroed = true;
          m_armIsResetting = false;
        }
>>>>>>> 7220e06 (Merging Arm Control subsystem to main (#20))
=======
>>>>>>> 70353eb (Dashboard pre work)
      }
    }
  }

  @Override
  public void initDashboard() {
    SmartDashboard.putNumber("Arm/Rotation/DesiredDegres", m_desiredAngleDeg);
    SmartDashboard.putNumber("Arm/Rotation/CurrentDegres", m_currentAngleDeg);
    SmartDashboard.putNumber("Arm/Rotation/AMPs_L", m_rotationMotor_L.getOutputCurrent());
    SmartDashboard.putNumber("Arm/Rotation/AMPs_R", m_rotationMotor_R.getOutputCurrent());

    SmartDashboard.putNumber("Arm/Extension/DesiredInches", m_desiredExtensionInches);
    SmartDashboard.putNumber("Arm/Extension/CurrentInches", m_currentExtInches);
    SmartDashboard.putNumber("Arm/Extension/CurrentDegrees", m_extensionEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Extension/AMPs_L", m_extensionMotor_L.getOutputCurrent());

    SmartDashboard.putBoolean("Arm/Extension/Resetting", m_armIsResetting);
    SmartDashboard.putBoolean("Arm/Extension/Zeroed", m_armZeroed);
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Arm/Rotation/DesiredDegres", m_desiredAngleDeg);
    SmartDashboard.putNumber("Arm/Rotation/CurrentDegres", m_currentAngleDeg);
    SmartDashboard.putNumber("Arm/Rotation/AMPs_L", m_rotationMotor_L.getOutputCurrent());
    SmartDashboard.putNumber("Arm/Rotation/AMPs_R", m_rotationMotor_R.getOutputCurrent());

    SmartDashboard.putNumber("Arm/Extension/DesiredInches", m_desiredExtensionInches);
    SmartDashboard.putNumber("Arm/Extension/CurrentInches", m_currentExtInches);
    SmartDashboard.putNumber("Arm/Extension/CurrentDegrees", m_extensionEncoder.getPosition());
    SmartDashboard.putNumber("Arm/Extension/AMPs_L", m_extensionMotor_L.getOutputCurrent());

    SmartDashboard.putBoolean("Arm/Extension/Resetting", m_armIsResetting);
    SmartDashboard.putBoolean("Arm/Extension/Zeroed", m_armZeroed);

    Pose3d ArmFramePose =
        new Pose3d(
            RobotConstants.kArmOffsetXMeters,
            RobotConstants.kArmOffsetYMeters,
            RobotConstants.kArmOffsetZMeters,
            new Rotation3d(
                Units.degreesToRadians(RobotConstants.kArmRotationRollOffsetDeg),
                Units.degreesToRadians(
                    RobotConstants.kArmRotationPitchOffsetDeg + m_currentAngleDeg),
                Units.degreesToRadians(RobotConstants.kArmRotationRollOffsetDeg)));

    Pose3d ArmExtPose =
        new Pose3d(
            RobotConstants.kArmOffsetXMeters
                + ((Units.inchesToMeters(m_currentExtInches)
                        + RobotConstants.kArmExtensionOffMeters)
                    * Math.cos(Units.degreesToRadians(m_currentAngleDeg))),
            RobotConstants.kArmOffsetYMeters,
            RobotConstants.kArmOffsetZMeters
                - ((Units.inchesToMeters(m_currentExtInches)
                        + RobotConstants.kArmExtensionOffMeters)
                    * Math.sin(Units.degreesToRadians(m_currentAngleDeg))),
            new Rotation3d(
                Units.degreesToRadians(RobotConstants.kArmRotationRollOffsetDeg),
                Units.degreesToRadians(
                    RobotConstants.kArmRotationPitchOffsetDeg + m_currentAngleDeg),
                Units.degreesToRadians(RobotConstants.kArmRotationRollOffsetDeg)));

    // RobotConstants.kArmOffsetToFrame.getRotation());

    poseArrayPublisher.set(new Pose3d[] {ArmFramePose, ArmExtPose});
  }

  @Override
  public DashboardUses getDashboardUses() {
    return DashboardUses.SHORT_INTERVAL;
  }

  private void updateSimValues() {
    if (m_currentAngleDeg < m_desiredAngleDeg) {
      m_currentAngleDeg += 0.5;
    } else if (m_currentAngleDeg > m_desiredAngleDeg) {
      m_currentAngleDeg -= 0.5;
    }

    if (m_currentExtInches < m_desiredExtensionInches) {
      m_currentExtInches += 0.1;
    } else if (m_currentExtInches > m_desiredExtensionInches) {
      m_currentExtInches -= 0.1;
    }

    // Clamp values for stable simulation
    m_currentAngleDeg =
        MathUtil.clamp(
            m_currentAngleDeg, ArmConstants.Rotation.kMinRotDeg, ArmConstants.Rotation.kMaxRotDeg);
    m_currentExtInches =
        MathUtil.clamp(
            m_currentExtInches,
            ArmConstants.Extension.kMinExtInches,
            ArmConstants.Extension.kMaxExtInches);
  }

  /** Controlling the Arm */
  public Command controlCommand(DoubleSupplier newRotValue, DoubleSupplier newExtValue) {
    return run(
        () -> {
          // Make the arm move
          this.control(newRotValue.getAsDouble(), newExtValue.getAsDouble());
        });
  }

  public void control(double newRot, double newExt) {
    this.rotate(newRot);
    this.extend(newExt);
  }
}
