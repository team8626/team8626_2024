// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.rotation;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Dashboard.ImplementDashboard;

/** Add your docs here. */
public interface ArmRotationIO extends ImplementDashboard {

    @AutoLog
    public class ArmRotationInputs {
        public double currentAngleDeg = 0;
        public boolean atSetpoint;
        public double leftMotorAmperage;
        public double rightMotorAmperage;
    }

    public default void updateInputs(ArmRotationInputs inputs) {}
}
