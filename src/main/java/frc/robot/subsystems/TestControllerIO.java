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
public class TestControllerIO implements ControllerOI {
  private final CommandXboxController m_testController =
      new CommandXboxController(Constants.OperatorConstants.kTestControllerPort);

  @Override
  public Runnable logOutputs() {
    return () -> {
      Logger.recordOutput("TestController/LeftX", getLeftX());
      Logger.recordOutput("TestController/LeftY", getLeftY());
      Logger.recordOutput("TestController/RightX", getRightX());
      Logger.recordOutput("TestController/RightY", getRightY());
      Logger.recordOutput("TestController/A", a().getAsBoolean());
      Logger.recordOutput("TestController/B", b().getAsBoolean());
      Logger.recordOutput("TestController/X", x().getAsBoolean());
      Logger.recordOutput("TestController/Y", y().getAsBoolean());
      Logger.recordOutput("TestController/LeftBumper", leftBumper().getAsBoolean());
      Logger.recordOutput("TestController/RightBumper", rightBumper().getAsBoolean());
      Logger.recordOutput("TestController/LeftTrigger", leftTrigger().getAsBoolean());
      Logger.recordOutput("TestController/RightTrigger", rightTrigger().getAsBoolean());
      Logger.recordOutput("TestController/POVUp", povUp().getAsBoolean());
      Logger.recordOutput("TestController/POVDown", povDown().getAsBoolean());
      Logger.recordOutput("TestController/POVLeft", povLeft().getAsBoolean());
      Logger.recordOutput("TestController/POVRight", povRight().getAsBoolean());
    };
  }

  @Override
  public double getLeftX() {
    return m_testController.getLeftX();
  }

  @Override
  public double getLeftY() {
    return m_testController.getLeftY();
  }

  @Override
  public double getRightX() {
    return m_testController.getRightX();
  }

  @Override
  public double getRightY() {
    return m_testController.getRightY();
  }

  @Override
  public Trigger a() {
    return m_testController.a();
  }

  public Trigger b() {
    return m_testController.b();
  }

  public Trigger x() {
    return m_testController.x();
  }

  public Trigger y() {
    return m_testController.y();
  }

  public Trigger leftBumper() {
    return m_testController.leftBumper();
  }

  public Trigger rightBumper() {
    return m_testController.rightBumper();
  }

  public Trigger leftTrigger() {
    return m_testController.leftTrigger();
  }

  public Trigger rightTrigger() {
    return m_testController.rightTrigger();
  }

  public Trigger povUp() {
    return m_testController.povUp();
  }

  public Trigger povDown() {
    return m_testController.povDown();
  }

  public Trigger povLeft() {
    return m_testController.povLeft();
  }

  public Trigger povRight() {
    return m_testController.povRight();
  }

  @Override
  public GenericHID getHID() {
    return m_testController.getHID();
  }

  @Override
  public double getRawAxis(int axis) {
    return m_testController.getRawAxis(axis);
  }
}
