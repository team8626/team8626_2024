// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;



import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Dashboard.DashboardUses;
import frc.robot.subsystems.Dashboard.ImplementDashboard;

public class ClimberSubsystem extends SubsystemBase implements ImplementDashboard {
 
private static CANSparkMax m_motorLeft = new CANSparkMax(ClimberConstants.kClimberMotorLeftPort, MotorType.kBrushless);
private static CANSparkMax m_motorRight = new CANSparkMax(ClimberConstants.kClimberMotorRightPort, MotorType.kBrushless);


  public ClimberSubsystem() {
    // TODO: Need this?
    // m_motorLeft.restoreFactoryDefaults();
    // m_motorRight.restoreFactoryDefaults();
  }

public void setMotors(double output) {
  m_motorLeft.set(output);
  m_motorRight.set(output);
}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initDashboard() {}

  @Override
  public void updateDashboard() {
  }

  @Override
  public DashboardUses getDashboardUses() {  
    return DashboardUses.LONG_INTERVAL;
}
}
