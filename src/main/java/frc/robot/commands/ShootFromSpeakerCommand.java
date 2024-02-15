// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.commands.SetArmCommand;
<<<<<<< HEAD
<<<<<<< HEAD
import frc.robot.subsystems.intake.IntakeSubsystem;
=======
>>>>>>> 0dbf64e (Autonomous Command Frames and Cleanup)
=======
import frc.robot.subsystems.intake.IntakeSubsystem;
>>>>>>> 4505236 (fix: ShooterCommand Prototype in auto commands)
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ShooterCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFromSpeakerCommand extends SequentialCommandGroup {

<<<<<<< HEAD
<<<<<<< HEAD
  public ShootFromSpeakerCommand(
      IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    addCommands(
        new SetArmCommand(arm, ArmConstants.Presets.kShootSpeaker_0ft),
        new ShooterCommand(intake, shooter, ShooterConstants.kShootFromSpeakerRPM));
=======
  public ShootFromSpeakerCommand(ShooterSubsystem shooter, ArmSubsystem arm) {
    addCommands(
        new SetArmCommand(arm, ArmConstants.Presets.kShootSpeaker_0ft),
        new ShooterCommand(shooter, ShooterConstants.kSpeakerShootSpeed));
>>>>>>> 0dbf64e (Autonomous Command Frames and Cleanup)
=======
  public ShootFromSpeakerCommand(
      IntakeSubsystem intake, ShooterSubsystem shooter, ArmSubsystem arm) {
    addCommands(
        new SetArmCommand(arm, ArmConstants.Presets.kShootSpeaker_0ft),
        new ShooterCommand(intake, shooter, ShooterConstants.kShootFromSpeakerRPM));
>>>>>>> 4505236 (fix: ShooterCommand Prototype in auto commands)
  }
}
