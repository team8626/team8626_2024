// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates.IntakeStatus;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.preset.Presets;

public class IntakeCommand extends Command {
  private IntakeSubsystem m_intake;
  private ArmRotationSubsystem m_armRot = null;
  private ArmExtensionSubsystem m_armExt = null;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intake) {
    addRequirements(intake);
    m_intake = intake;

    // andThen(new IntakeAdjustmentCommand(intake));
    // .onlyIf(() -> !m_intake.isFull() && m_intake.limitReached()));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** Creates a new IntakeCommand. */
  public IntakeCommand(
      IntakeSubsystem intake, ArmRotationSubsystem armRot, ArmExtensionSubsystem armExt) {
    addRequirements(intake);
    m_intake = intake;
    m_armRot = armRot;
    m_armExt = armExt;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.start(IntakeConstants.kSpeed_Intake);

    m_intake.setStatus(IntakeStatus.INTAKING);
    LEDSubsystem.setMode(LedMode.INTAKING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.isFull()) {

      m_intake.setSpeed(IntakeConstants.kSpeed_Coast);
      m_intake.setStatus(IntakeStatus.COASTING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();

    // Constructed with Rotation and Extension, reset arm to "Stow"
    if ((m_armRot != null) && (m_armExt != null)) {
      m_armRot.setAngleDeg(Presets.kStow.getRotDegrees());
      m_armExt.setLengthInches(Presets.kStow.getExtInches());
    }
    m_intake.setStatus(IntakeStatus.IDLE);
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.limitReached();
  }
}
