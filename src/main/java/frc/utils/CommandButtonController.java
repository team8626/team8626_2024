// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for using button box with triggers */
public class CommandButtonController extends CommandGenericHID {
  private final Joystick m_hid;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandButtonController(int port) {
    super(port);
    m_hid = new Joystick(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public Joystick getHID() {
    return m_hid;
  }

  /**
   * Constructs event instance around the button's digital signal.
   *
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   */
  public Trigger button_1() {
    return m_hid
        .button(1, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_2() {
    return m_hid
        .button(2, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_3() {
    return m_hid
        .button(3, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_4() {
    return m_hid
        .button(4, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_5() {
    return m_hid
        .button(5, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_6() {
    return m_hid
        .button(6, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_7() {
    return m_hid
        .button(7, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_8() {
    return m_hid
        .button(8, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }

  public Trigger button_9() {
    return m_hid
        .button(9, CommandScheduler.getInstance().getDefaultButtonLoop())
        .castTo(Trigger::new);
  }
}
