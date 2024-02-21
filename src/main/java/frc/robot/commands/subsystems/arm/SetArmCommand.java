// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Presets.Preset;
import frc.robot.subsystems.arm.extension.ArmExtensionSubsystem;
import frc.robot.subsystems.arm.rotation.ArmRotationSubsystem;
import java.util.function.Supplier;

public class SetArmCommand extends Command {
  private ArmRotationSubsystem m_armRot;
  private ArmExtensionSubsystem m_armExt;
  private Supplier<Preset> m_preset;
  private boolean m_goodAngle = false;

  private Timer m_timer = new Timer();

  double m_desiredExtensionInches, m_desiredAngleDegrees;

  // /*
  //  * Set arm from specific values
  //  */
  // public SetArmCommand(
  //     ArmRotationSubsystem armRot,
  //     ArmExtensionSubsystem armExt,
  //     double desiredExtensionInches,
  //     double desiredAngleDegrees) {

  //   m_armRot = armRot;
  //   m_armExt = armExt;

  //   m_desiredExtensionInches = desiredExtensionInches;
  //   m_desiredAngleDegrees = desiredAngleDegrees;

  //   addRequirements(armRot);
  //   addRequirements(armExt);

  //   setName("Set Arm Command");
  // }

  /*
   * Set arm from a preset
   */
  public SetArmCommand(
      ArmRotationSubsystem armRot, ArmExtensionSubsystem armExt, Supplier<Preset> desiredState) {
    m_armExt = armExt;
    m_armRot = armRot;
    m_preset = desiredState;

    // m_desiredExtensionInches = desiredState.getExtInches();
    // m_desiredAngleDegrees = desiredState.getRotDegrees();

    addRequirements(armRot);
    addRequirements(armExt);

    setName("Set Arm Command");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();

    System.out.printf("[SetArmCommand] Preset: %s\n", m_preset.get().getString());

    m_armRot.setAngleDeg(m_preset.get().getRotDegrees());
    m_goodAngle = false;

    System.out.printf("[SetArmCommand] New rot: %f\n", m_preset.get().getRotDegrees());
    // m_armExt.setLengthInches(m_preset.getExtInches());
  }

  @Override
  public void execute() {
    if (MathUtil.isNear(m_preset.get().getRotDegrees(), m_armRot.getRotDegrees(), 35)) {
      m_armExt.setLengthInches(m_preset.get().getExtInches());
      // System.out.printf(
      //     "[SetArmCommand] New ext: %f (Current Rot: %f want %f)\n",
      //     m_preset.get().getExtInches(), m_armRot.getRotDegrees(),
      // m_preset.get().getRotDegrees());
    } else {
      // System.out.printf(
      //     "[SetArmCommand] Delay ext: %f (Current Rot: %f waiting for %f +/-20)\n",
      //     m_preset.get().getExtInches(), m_armRot.getRotDegrees(),
      // m_preset.get().getRotDegrees());
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_goodAngle = false;
    m_timer.stop();

    System.out.println("[SetArmCommand] Ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean retval = false;

    if (m_armRot.atSetpoint() && m_armExt.atSetpoint()) {
      m_timer.start();
      if (m_timer.hasElapsed(1)) {
        retval = true;
      }
    }
    return retval;
  }
}
