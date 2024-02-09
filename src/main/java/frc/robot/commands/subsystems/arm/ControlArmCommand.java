package frc.robot.commands.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class ControlArmCommand extends Command {
  private ArmSubsystem m_arm;
  private double m_rot = 0;
  private double m_ext = 0;

  public ControlArmCommand(DoubleSupplier newRot, DoubleSupplier newExt, ArmSubsystem arm) {
    m_arm = arm;
    m_rot = newRot.getAsDouble();
    m_ext = newExt.getAsDouble();
    addRequirements(arm);
  }

  @Override
  public void execute() {
    m_arm.rotate(m_rot);
    m_arm.extend(m_ext);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
