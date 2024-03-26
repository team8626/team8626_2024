// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.subsystems.intake.IntakeAdjustmentCommand;
import frc.robot.commands.subsystems.intake.IntakeCommand;
import frc.robot.subsystems.LEDs.LEDConstants.LedMode;
import frc.robot.subsystems.LEDs.LEDSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setArmMotorsMode(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.setArmMotorsMode(IdleMode.kBrake);
    m_autonomousCommand =
        // Force intake and adjust preloaded NOTE
        new IntakeCommand(m_robotContainer.m_intake)
            .andThen(new IntakeAdjustmentCommand(m_robotContainer.m_intake))
            .withTimeout(1)
            .andThen(m_robotContainer.getAutonomousCommand());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // The origin is always blue. When our alliance is red, X and Y need to be inverted
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      m_robotContainer.invert = -1;
    } else {
      m_robotContainer.invert = 1;
    }

    m_robotContainer.setArmMotorsMode(IdleMode.kBrake);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    LEDSubsystem.setMode(LedMode.DEFAULT);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
