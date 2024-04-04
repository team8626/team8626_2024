// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ControllerOI;
import frc.robot.subsystems.swervedrive.Constants;
import frc.utils.CommandButtonController;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ButtonBoxOI implements ControllerOI {
  private final CommandButtonController m_buttonBox =
      new CommandButtonController(Constants.OperatorConstants.kButtonBoxPort);

  @Override
  public Runnable logOutputs() {
    return () -> {
      Logger.recordOutput("ButtonBox/Button1", button_1().getAsBoolean());
      Logger.recordOutput("ButtonBox/Button2", button_2().getAsBoolean());
      Logger.recordOutput("ButtonBox/Button3", button_3().getAsBoolean());
      Logger.recordOutput("ButtonBox/Button4", button_4().getAsBoolean());
      Logger.recordOutput("ButtonBox/Button5", button_5().getAsBoolean());
      Logger.recordOutput("ButtonBox/Button6", button_6().getAsBoolean());
      Logger.recordOutput("ButtonBox/Button7", button_7().getAsBoolean());
      Logger.recordOutput("ButtonBox/Button8", button_8().getAsBoolean());
      Logger.recordOutput("ButtonBox/Button9", button_9().getAsBoolean());
    };
  }

  public Trigger button_1() {
    return m_buttonBox.button_1();
  }

  public Trigger button_2() {
    return m_buttonBox.button_2();
  }

  public Trigger button_3() {
    return m_buttonBox.button_3();
  }

  public Trigger button_4() {
    return m_buttonBox.button_4();
  }

  public Trigger button_5() {
    return m_buttonBox.button_5();
  }

  public Trigger button_6() {
    return m_buttonBox.button_6();
  }

  public Trigger button_7() {
    return m_buttonBox.button_7();
  }

  public Trigger button_8() {
    return m_buttonBox.button_8();
  }

  public Trigger button_9() {
    return m_buttonBox.button_9();
  }

  @Override
  public GenericHID getHID() {
    return m_buttonBox.getHID();
  }
}
