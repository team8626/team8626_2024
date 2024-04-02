// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates.IntakeStatus;

public class IntakeSubsystem extends SubsystemBase{
  IntakeIO m_io = new IntakeIOReal();
  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io){
    m_io = io;
  }
  public void setSpeed(double speed){
    m_io.setSpeed(speed);
  }
  public void start(){
    m_io.start();
  }
  public void start(double speed){
    m_io.start(speed);
  }
  public void stop(){
    m_io.stop();
  }
  public void setStatus(IntakeStatus status){
    m_io.setStatus(status);
  }
  public boolean isFull(){
    return m_io.isFull();
  }
  public boolean isEmpty(){
    return m_io.isEmpty();
  }
  public boolean limitReached(){
    return m_io.limitReached();
  }
  public IntakeIO getIO(){
    return m_io;
  }
  @Override
  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.processInputs("IntakeSubsystem", inputs);
  }
}
