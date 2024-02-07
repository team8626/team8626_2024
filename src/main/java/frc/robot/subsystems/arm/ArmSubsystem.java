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
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class ArmSubsystem extends SubsystemBase implements ImplementDashboard {
  /** Create Motors * */
  private final CANSparkMax m_extensionMotor_L;
  // private final CANSparkMax m_extensionMotor_R;
  private final CANSparkMax m_rotationMotor_L;
  private final CANSparkMax m_rotationMotor_R;

  /** Create Sensors * */
  private final AbsoluteEncoder m_extensionEncoder;

  private final AbsoluteEncoder m_rotationEncoder;
  private final SparkPIDController m_extensionPIDController;
  private final SparkPIDController m_rotationPIDController;

  /** Class States * */
  private double m_desiredAngleDeg = 0;

  private double m_desiredExtensionInches = 0;

  private double m_currentAngleDeg = 0;
  private double m_currentExtInches = 0;

  /** Poses publisher for AdvantageScope * */
  StructArrayPublisher<Pose3d> poseArrayPublisher =
      NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

  /** Creates a new ArmExtensionSubsystem. */
  public ArmSubsystem() {
    /** Motors * */
    m_extensionMotor_L =
        new CANSparkMax(ArmConstants.Extension.extensionCANID_L, MotorType.kBrushless);
    // m_extensionMotor_R = new CANSparkMax(ArmConstants.extensionCANID_R, MotorType.kBrushless);
    m_rotationMotor_L =
        new CANSparkMax(ArmConstants.Rotation.rotationCANID_L, MotorType.kBrushless);
    m_rotationMotor_R =
        new CANSparkMax(ArmConstants.Rotation.rotationCANID_R, MotorType.kBrushless);

    /** Reset & Initialize Controllers * */
    m_extensionMotor_L.restoreFactoryDefaults();
    // m_extensionMotor_R.restoreFactoryDefaults();
    // m_extensionMotor_R.follow(m_extensionMotor_L, true);

    m_rotationMotor_L.restoreFactoryDefaults();
    m_rotationMotor_R.restoreFactoryDefaults();
    m_rotationMotor_R.follow(m_rotationMotor_L, true);

    // Setup encoders and PID controllers for SPARKMAX.
    m_extensionEncoder = m_extensionMotor_L.getAbsoluteEncoder(Type.kDutyCycle);
    m_extensionPIDController = m_extensionMotor_L.getPIDController();
    m_extensionPIDController.setFeedbackDevice(m_extensionEncoder);

    // Extension Soft Limits
    m_extensionMotor_L.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_extensionMotor_L.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_extensionMotor_L.setSoftLimit(
        SoftLimitDirection.kReverse, ArmConstants.Extension.kExtensionMinDeg);
    m_extensionMotor_L.setSoftLimit(
        SoftLimitDirection.kForward, ArmConstants.Extension.kExtensionMaxDeg);

    m_rotationEncoder = m_rotationMotor_L.getAbsoluteEncoder(Type.kDutyCycle);
    m_rotationPIDController = m_rotationMotor_L.getPIDController();
    m_rotationPIDController.setFeedbackDevice(m_rotationEncoder);

    // Rotation Soft Limits
    m_rotationMotor_L.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_rotationMotor_L.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_rotationMotor_L.setSoftLimit(
        SoftLimitDirection.kReverse, ArmConstants.Rotation.kRotationMinDeg);
    m_rotationMotor_L.setSoftLimit(
        SoftLimitDirection.kForward, ArmConstants.Rotation.kRotationMaxDeg);

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
    m_rotationMotor_L.setInverted(false);

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

    /** Initialize the Subsystem * */
    if (RobotBase.isReal()) {
      m_desiredAngleDeg = m_rotationEncoder.getPosition();
      m_desiredExtensionInches = getExtensionInchesFromDeg(m_extensionEncoder.getPosition());
    }
  }

  /*
   *
   */
  public void setAngleDeg(double newAngleDeg) {
    m_desiredAngleDeg = newAngleDeg;
  }

  public double getAngleDeg() {
    return m_currentAngleDeg;
  }

  /*
   *
   */
  public void setLength(int newLengthInches) {
    m_desiredExtensionInches = newLengthInches;
  }

  /*
   *
   */
  public double getLengthInches() {
    return m_currentExtInches;
  }

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
    // TODO: Is this Angle allowed based on current extension?
    m_rotationPIDController.setReference(m_desiredAngleDeg, ControlType.kPosition);

    // TODO: Is this Extension allowed based on current rotation?
    m_extensionPIDController.setReference(
        this.getDegreesFromExtensionInches(m_desiredExtensionInches), ControlType.kPosition);

    if (RobotBase.isReal()) {
      m_currentAngleDeg = m_rotationEncoder.getPosition();
      m_currentExtInches = this.getExtensionInchesFromDeg(m_extensionEncoder.getPosition());
      ;
    } else {
      updateSimValues();
    }
  }

  @Override
  public void initDashboard() {
    SmartDashboard.putNumber("Desired_Rot", m_desiredAngleDeg);
    SmartDashboard.putNumber("Desired_Ext", m_desiredExtensionInches);
    SmartDashboard.putNumber("Current_Rot", m_currentAngleDeg);
    SmartDashboard.putNumber("Current_Ext", m_currentExtInches);
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Desired_Rot", m_desiredAngleDeg);
    SmartDashboard.putNumber("Desired_Ext", m_desiredExtensionInches);
    SmartDashboard.putNumber("Current_Rot", m_currentAngleDeg);
    SmartDashboard.putNumber("Current_Ext", m_currentExtInches);

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
      m_currentAngleDeg += 1;
    } else if (m_currentAngleDeg > m_desiredAngleDeg) {
      m_currentAngleDeg -= 1;
    }

    if (m_currentExtInches < m_desiredExtensionInches) {
      m_currentExtInches += 0.5;
    } else if (m_currentExtInches > m_desiredExtensionInches) {
      m_currentExtInches -= 0.5;
    }
  }
}
