// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ControllerOI;
import frc.robot.subsystems.swervedrive.Constants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class XboxControllerOI implements ControllerOI {
  private final CommandXboxController m_xboxController =
      new CommandXboxController(Constants.OperatorConstants.kXboxControllerPort);

  @Override
  public Runnable logOutputs() {
    return () -> {
      Logger.recordOutput("XboxController/LeftX", getLeftX());
      Logger.recordOutput("XboxController/LeftY", getLeftY());
      Logger.recordOutput("XboxController/RightX", getRightX());
      Logger.recordOutput("XboxController/RightY", getRightY());
      Logger.recordOutput("XboxController/A", a().getAsBoolean());
      Logger.recordOutput("XboxController/B", b().getAsBoolean());
      Logger.recordOutput("XboxController/X", x().getAsBoolean());
      Logger.recordOutput("XboxController/Y", y().getAsBoolean());
      Logger.recordOutput("XboxController/LeftBumper", leftBumper().getAsBoolean());
      Logger.recordOutput("XboxController/RightBumper", rightBumper().getAsBoolean());
      Logger.recordOutput("XboxController/LeftTrigger", leftTrigger().getAsBoolean());
      Logger.recordOutput("XboxController/RightTrigger", rightTrigger().getAsBoolean());
      Logger.recordOutput("XboxController/POVUp", povUp().getAsBoolean());
      Logger.recordOutput("XboxController/POVDown", povDown().getAsBoolean());
      Logger.recordOutput("XboxController/POVLeft", povLeft().getAsBoolean());
      Logger.recordOutput("XboxController/POVRight", povRight().getAsBoolean());
    };
  }

  @Override
  public double getLeftX() {
    return m_xboxController.getLeftX();
  }

  @Override
  public double getLeftY() {
    return m_xboxController.getLeftY();
  }

  @Override
  public double getRightX() {
    return m_xboxController.getRightX();
  }

  @Override
  public double getRightY() {
    return m_xboxController.getRightY();
  }

  @Override
  public Trigger a() {
    return m_xboxController.a();
  }

  public Trigger b() {
    return m_xboxController.b();
  }

  public Trigger x() {
    return m_xboxController.x();
  }

  public Trigger y() {
    return m_xboxController.y();
  }

  public Trigger leftBumper() {
    return m_xboxController.leftBumper();
  }

  public Trigger rightBumper() {
    return m_xboxController.rightBumper();
  }

  public Trigger leftTrigger() {
    return m_xboxController.leftTrigger();
  }

  public Trigger rightTrigger() {
    return m_xboxController.rightTrigger();
  }

  public Trigger povUp() {
    return m_xboxController.povUp();
  }

  public Trigger povDown() {
    return m_xboxController.povDown();
  }

  public Trigger povLeft() {
    return m_xboxController.povLeft();
  }

  public Trigger povRight() {
    return m_xboxController.povRight();
  }

  @Override
  public GenericHID getHID() {
    return m_xboxController.getHID();
  }

  @Override
  public double getRawAxis(int axis) {
    return m_xboxController.getRawAxis(axis);
  }
}
