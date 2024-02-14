// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
<<<<<<< HEAD
<<<<<<< HEAD
import frc.robot.subsystems.arm.ArmConstants.Preset;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetArmCommand extends Command {
  ArmSubsystem m_arm;
  double m_desiredExtensionInches, m_desiredAngleDegrees;

  /*
   * Set arm from specific values
   */
  public SetArmCommand(
      ArmSubsystem arm, double desiredExtensionInches, double desiredAngleDegrees) {
=======
import frc.robot.subsystems.arm.ArmSubsystem;
=======
>>>>>>> 0dbf64e (Autonomous Command Frames and Cleanup)
import frc.robot.subsystems.arm.ArmConstants.Preset;
import frc.robot.subsystems.arm.ArmSubsystem;

public class SetArmCommand extends Command {
  ArmSubsystem m_arm;
  double m_desiredExtensionInches, m_desiredAngleDegrees;

  /*
   * Set arm from specific values
<<<<<<< HEAD
  */
  public SetArmCommand(ArmSubsystem arm, double desiredExtensionInches, double desiredAngleDegrees) {
>>>>>>> a2e5544 (Set Arm Command)
=======
   */
  public SetArmCommand(
      ArmSubsystem arm, double desiredExtensionInches, double desiredAngleDegrees) {
>>>>>>> 0dbf64e (Autonomous Command Frames and Cleanup)
    m_arm = arm;
    m_desiredExtensionInches = desiredExtensionInches;
    m_desiredAngleDegrees = desiredAngleDegrees;

    addRequirements(arm);
    setName("Set Arm Command");
<<<<<<< HEAD
<<<<<<< HEAD
  }

  /*
   * Set arm from a preset
   */
  public SetArmCommand(ArmSubsystem arm, Preset desiredState) {
    m_arm = arm;
    m_desiredExtensionInches = desiredState.getExtInches();
    m_desiredAngleDegrees = desiredState.getRotDegrees();

    addRequirements(arm);
    setName("Set Arm Command");
=======
    
>>>>>>> a2e5544 (Set Arm Command)
=======
  }

  /*
   * Set arm from a preset
   */
  public SetArmCommand(ArmSubsystem arm, Preset desiredState) {
    m_arm = arm;
    m_desiredExtensionInches = desiredState.getExtInches();
    m_desiredAngleDegrees = desiredState.getRotDegrees();

    addRequirements(arm);
    setName("Set Arm Command");
>>>>>>> 13c2f89 (Set Arm Command Preset Functionality)
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setAngleDeg(m_desiredAngleDegrees);
    m_arm.setLengthInches(m_desiredExtensionInches);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atAngleSetpoint() && m_arm.atExtensionSetpoint();
  }
}
