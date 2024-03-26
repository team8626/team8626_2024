// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.rotation;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;
import frc.robot.subsystems.preset.Presets;
import java.util.function.DoubleSupplier;

public class ArmRotationSubsystem extends SubsystemBase implements ImplementDashboard {

  /** Create Motors * */
  private final CANSparkMax m_rotationMotor_L;

  private final CANSparkMax m_rotationMotor_R;

  /** Create Sensors * */
  private final AbsoluteEncoder m_rotationEncoder;

  private final SparkPIDController m_rotationPIDController;

  /** Class States * */
  private double m_desiredAngleDeg = 0;

  private double m_currentAngleDeg = 180;

  /** Poses publisher for AdvantageScope * */
  // StructArrayPublisher<Pose3d> poseArrayPublisher =
  //     NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray",
  // Pose3d.struct).publish();

  private double m_kRotationP = RotConstants.kRotP;

  private double m_kRotationI = RotConstants.kRotI;
  private double m_kRotationD = RotConstants.kRotD;
  private double m_kRotationFF = RotConstants.kRotFF;
  private double m_kRotationMaxOutput = RotConstants.kRotMaxOutput;
  private double m_kRotationMinOutput = RotConstants.kRotMinOutput;

  /** Creates a new ArmExtensionSubsystem. */
  public ArmRotationSubsystem() {
    // Reset & Initialize Controllers
    m_rotationMotor_L = new CANSparkMax(RotConstants.rotationCANID_L, MotorType.kBrushless);
    m_rotationMotor_R = new CANSparkMax(RotConstants.rotationCANID_R, MotorType.kBrushless);

    m_rotationMotor_L.restoreFactoryDefaults();
    m_rotationMotor_R.restoreFactoryDefaults();

    m_rotationMotor_R.follow(m_rotationMotor_L, true);
    m_rotationMotor_L.setInverted(false);

    m_rotationEncoder = m_rotationMotor_L.getAbsoluteEncoder(Type.kDutyCycle);

    m_rotationPIDController = m_rotationMotor_L.getPIDController();
    m_rotationPIDController.setFeedbackDevice(m_rotationEncoder);

    // Rotation Soft Limits
    m_rotationMotor_L.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_rotationMotor_L.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_rotationMotor_L.setSoftLimit(SoftLimitDirection.kReverse, RotConstants.kMinRotDeg);
    m_rotationMotor_L.setSoftLimit(SoftLimitDirection.kForward, RotConstants.kMaxRotDeg);

    // Apply position and velocity conversion factors for the turning encoder.
    // We want these in degrees and degrees per second
    m_rotationEncoder.setPositionConversionFactor(RotConstants.kRotationEncoderPositionFactorDeg);
    m_rotationEncoder.setVelocityConversionFactor(RotConstants.kRotationEncoderVelocityFactorDeg);
    // m_rotationEncoder.setZeroOffset(m_rotationEncoder.getZeroOffset() + 180);
    m_rotationEncoder.setInverted(false);

    // Set the PID gains for the motor.
    m_rotationPIDController.setP(m_kRotationP);
    m_rotationPIDController.setI(m_kRotationI);
    m_rotationPIDController.setD(m_kRotationD);
    m_rotationPIDController.setFF(m_kRotationFF);
    m_rotationPIDController.setOutputRange(m_kRotationMinOutput, m_kRotationMaxOutput);

    // Idle Modes and max current
    m_rotationMotor_L.setIdleMode(IdleMode.kBrake);
    m_rotationMotor_L.setSmartCurrentLimit(RotConstants.kCurrentLimit);

    // DO NOT WRITE (BURNFLASH) TO MOTOR SINCE WE UPDATED THE ZERO OFFSET!!!!

    // FORCE DASHBOARD UPDATE (Get values)
    updateDashboard();

    if (Robot.isReal()) {
      m_desiredAngleDeg = Presets.kStow.getRotDegrees();

    } else if (Robot.isSimulation()) {
      m_desiredAngleDeg = Presets.kStart.getRotDegrees();
      m_currentAngleDeg = m_desiredAngleDeg;
    }
  }

  /**
   * Set Arm Rotation to a specific rotation in Degrees.
   *
   * @param newAngleDeg
   */
  public void setAngleDeg(double newAngleDeg) {
    m_desiredAngleDeg = newAngleDeg; // Value Clamped in periodic()
    System.out.printf("[ARM] New Angle: %f\n", m_desiredAngleDeg);
  }

  public void rotate(double newSpeed) {
    if (Robot.isReal()) {
      // Soft Limit...
      // Allow control only if rotation is in range or coming back to range
      if ((m_currentAngleDeg < (RotConstants.kMinRotDeg) && (newSpeed > 0))
          || (m_currentAngleDeg > (RotConstants.kMaxRotDeg) && (newSpeed < 0))
          || ((m_currentAngleDeg >= (RotConstants.kMinRotDeg)
              && (m_currentAngleDeg <= (RotConstants.kMaxRotDeg))))) {
        m_rotationPIDController.setReference(-newSpeed * 0.50, ControlType.kDutyCycle);
      }
    } else { // Simulation
      if (newSpeed > 0) {
        m_desiredAngleDeg += 1;
      } else if (newSpeed < 0) {
        m_desiredAngleDeg -= 1;
      }
    }
  }

  public boolean atSetpoint() {
    return MathUtil.isNear(m_desiredAngleDeg, m_currentAngleDeg, RotConstants.kAtAngleTolerance);
  }

  private boolean isInRange(double angleDegree) {
    boolean retval = true;
    if ((angleDegree < RotConstants.kMinRotDeg) || (angleDegree > RotConstants.kMaxRotDeg)) {
      retval = false;
    }
    return retval;
  }

  public void setMotorsMode(IdleMode mode) {
    m_rotationMotor_L.setIdleMode(mode);
    m_rotationMotor_R.setIdleMode(mode);
  }

  public double getRotDegrees() {
    return m_currentAngleDeg;
  }

  @Override
  public void periodic() {
    /** Check for value in Range * */
    m_desiredAngleDeg =
        MathUtil.clamp(m_desiredAngleDeg, RotConstants.kMinRotDeg, RotConstants.kMaxRotDeg);

    /** Update Current Arm Positions */
    if (Robot.isReal()) {
      m_currentAngleDeg = m_rotationEncoder.getPosition();
    } else {
      updateSimValues();
    }

    m_rotationPIDController.setReference(m_desiredAngleDeg, ControlType.kPosition);
  }

  @Override
  public void initDashboard() {
    SmartDashboard.putNumber("Arm/Rotation/P Gain", m_kRotationP);
    SmartDashboard.putNumber("Arm/Rotation/I Gain", m_kRotationI);
    SmartDashboard.putNumber("Arm/Rotation/D Gain", m_kRotationD);
    SmartDashboard.putNumber("Arm/Rotation/Feed Forward", m_kRotationFF);
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Arm/Rotation/DesiredDegres", m_desiredAngleDeg);
    SmartDashboard.putNumber("Arm/Rotation/CurrentDegres", m_currentAngleDeg);
    SmartDashboard.putNumber("Arm/Rotation/AMPs_L", m_rotationMotor_L.getOutputCurrent());
    SmartDashboard.putNumber("Arm/Rotation/AMPs_R", m_rotationMotor_R.getOutputCurrent());
    SmartDashboard.putBoolean("Arm/Rotation/atSetPoint", atSetpoint());

    double rotP = SmartDashboard.getNumber("Arm/Rotation/P Gain", RotConstants.kRotP);
    double rotI = SmartDashboard.getNumber("Arm/Rotation/I Gain", RotConstants.kRotI);
    double rotD = SmartDashboard.getNumber("Arm/Rotation/D Gain", RotConstants.kRotD);
    double rotFF = SmartDashboard.getNumber("Arm/Rotation/Feed Forward", RotConstants.kRotFF);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((rotP != m_kRotationP)) {
      m_rotationPIDController.setP(rotP);
      m_kRotationP = rotP;
    }
    if ((rotI != m_kRotationI)) {
      m_rotationPIDController.setD(rotI);
      m_kRotationI = rotI;
    }
    if ((rotD != m_kRotationD)) {
      m_rotationPIDController.setD(rotD);
      m_kRotationD = rotD;
    }
    if ((rotFF != m_kRotationFF)) {
      m_rotationPIDController.setFF(rotFF);
      m_kRotationFF = rotFF;
    }

    SmartDashboard.putNumber("Arm/Rotation/P Gain", m_kRotationP);
    SmartDashboard.putNumber("Arm/Rotation/I Gain", m_kRotationI);
    SmartDashboard.putNumber("Arm/Rotation/D Gain", m_kRotationD);
    SmartDashboard.putNumber("Arm/Rotation/Error", m_desiredAngleDeg - m_currentAngleDeg);
    SmartDashboard.putNumber("Arm/Rotation/Feed Forward", m_kRotationFF);

    // Pose3d ArmFramePose =
    //     new Pose3d(
    //         RobotConstants.kArmOffsetXMeters,
    //         RobotConstants.kArmOffsetYMeters,
    //         RobotConstants.kArmOffsetZMeters,
    //         new Rotation3d(
    //             Units.degreesToRadians(RobotConstants.kArmRotationRollOffsetDeg),
    //             Units.degreesToRadians(
    //                 RobotConstants.kArmRotationPitchOffsetDeg + m_currentAngleDeg),
    //             Units.degreesToRadians(RobotConstants.kArmRotationRollOffsetDeg)));

    // Pose3d ArmExtPose =
    //     new Pose3d(
    //         RobotConstants.kArmOffsetXMeters
    //             + ((Units.inchesToMeters(m_currentExtInches)
    //                     + RobotConstants.kArmExtensionOffMeters)
    //                 * Math.cos(Units.degreesToRadians(m_currentAngleDeg))),
    //         RobotConstants.kArmOffsetYMeters,
    //         RobotConstants.kArmOffsetZMeters
    //             - ((Units.inchesToMeters(m_currentExtInches)
    //                     + RobotConstants.kArmExtensionOffMeters)
    //                 * Math.sin(Units.degreesToRadians(m_currentAngleDeg))),
    //         new Rotation3d(
    //             Units.degreesToRadians(RobotConstants.kArmRotationRollOffsetDeg),
    //             Units.degreesToRadians(
    //                 RobotConstants.kArmRotationPitchOffsetDeg + m_currentAngleDeg),
    //             Units.degreesToRadians(RobotConstants.kArmRotationRollOffsetDeg)));

    // poseArrayPublisher.set(new Pose3d[] {ArmFramePose, ArmExtPose});
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
    // Clamp values for stable simulation
    m_currentAngleDeg =
        MathUtil.clamp(m_currentAngleDeg, RotConstants.kMinRotDeg, RotConstants.kMaxRotDeg);
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
    // this.rotate(newRot);
    // this.extend(newExt);
  }
}
