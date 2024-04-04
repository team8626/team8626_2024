// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public interface ControllerOI {

  public enum ControllerType {}

  // public class ControllerOutputLogger {
  //     public ControllerOutputLogger(BooleanSupplier... values) {
  //         BooleanSupplier[] m_values = values;
  //         Runnable logger = () -> {
  //             for(BooleanSupplier button : m_values) System.out.println(button.getAsBoolean());
  //         };
  //      new Notifier(logger).startPeriodic(0);
  //     }
  // }

  public default void createLogging() {
    Trigger teleopTrigger = new Trigger(() -> DriverStation.isTeleop());
    teleopTrigger.onTrue(new RunCommand(logOutputs()));
  }

  public default Runnable logOutputs() {
    return () -> System.out.println();
  }

  public GenericHID getHID();

  public default double getRawAxis(int axis) {
    return 0;
  }

  public default double getLeftX() {
    return 0;
  }

  public default double getLeftY() {
    return 0;
  }

  public default double getRightX() {
    return 0;
  }

  public default double getRightY() {
    return 0;
  }

  public default Trigger a() {
    return new Trigger(() -> false);
  }

  public default Trigger b() {
    return new Trigger(() -> false);
  }

  public default Trigger x() {
    return new Trigger(() -> false);
  }

  public default Trigger y() {
    return new Trigger(() -> false);
  }

  public default Trigger leftBumper() {
    return new Trigger(() -> false);
  }

  public default Trigger rightBumper() {
    return new Trigger(() -> false);
  }

  public default Trigger leftTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger rightTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger povUp() {
    return new Trigger(() -> false);
  }

  public default Trigger povDown() {
    return new Trigger(() -> false);
  }

  public default Trigger povLeft() {
    return new Trigger(() -> false);
  }

  public default Trigger povRight() {
    return new Trigger(() -> false);
  }

  public default Trigger button_1() {
    return new Trigger(() -> false);
  }

  public default Trigger button_2() {
    return new Trigger(() -> false);
  }

  public default Trigger button_3() {
    return new Trigger(() -> false);
  }

  public default Trigger button_4() {
    return new Trigger(() -> false);
  }

  public default Trigger button_5() {
    return new Trigger(() -> false);
  }

  public default Trigger button_6() {
    return new Trigger(() -> false);
  }

  public default Trigger button_7() {
    return new Trigger(() -> false);
  }

  public default Trigger button_8() {
    return new Trigger(() -> false);
  }

  public default Trigger button_9() {
    return new Trigger(() -> false);
  }
}
