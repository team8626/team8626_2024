// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.preset.Presets.Preset;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.Supplier;

public class SpinAndShootCommand extends Command {
  /** Creates a new ShooterCommand. */
  private ShooterSubsystem m_shooter;

  private IntakeSubsystem m_intake;

  private ArmRotationSubsystem m_armRot;
  private ArmExtensionSubsystem m_armExt;

  private Supplier<Preset> m_preset;

  private Timer m_timer = new Timer();

  public SpinAndShootCommand(
      IntakeSubsystem intake,
      ShooterSubsystem shooter,
      ArmRotationSubsystem armRot,
      ArmExtensionSubsystem armExt,
      Supplier<Preset> preset) {
    m_shooter = shooter;
    m_intake = intake;
    m_armRot = armRot;
    m_armExt = armExt;
    m_preset = preset;

    addRequirements(shooter, intake, armRot, armExt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_shooter.setRPM(m_preset.get().getBottomRPM(), m_preset.get().getTopRPM());
    m_shooter.start();
    m_armRot.setAngleDeg(m_preset.get().getRotDegrees());
    LEDSubsystem.setMode(LedMode.SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (MathUtil.isNear(m_preset.get().getRotDegrees(), m_armRot.getRotDegrees(), 50)) {
      m_armExt.setLengthInches(m_preset.get().getExtInches());
    }

    if (m_armRot.atSetpoint() && m_armExt.atSetpoint() && m_shooter.isAtSpeed()) {
      m_intake.start(IntakeConstants.kSpeed_Shoot);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_intake.stop();
    m_timer.stop();
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retval = false;

    if (m_intake.isEmpty()) {
      m_timer.start();

      if (m_timer.hasElapsed(1)) {
        retval = true;
      }
    }

    return retval;
  }
}
