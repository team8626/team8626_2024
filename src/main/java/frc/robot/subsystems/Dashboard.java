// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {

private static ArrayList<ImplementDashboard> dashboardShortUpdates = new ArrayList<ImplementDashboard>();
private static ArrayList<ImplementDashboard> dashboardLongUpdates = new ArrayList<ImplementDashboard>();

private static final boolean kEnableDashBoard = true;
	
private static final double kShortInterval = .02;
private static final double kLongInterval  = .5;
    

private double m_shortOldTime = 0.0;
private double m_longOldTime  = 0.0;   

// Use assignment types for updateDashboard implementation
public enum DashboardUses {
  SHORT_INTERVAL,
  LONG_INTERVAL
}

public static interface ImplementDashboard {

  public void initDashboard();
  public void updateDashboard();
  public DashboardUses getDashboardUses();
  
}

public Dashboard(ImplementDashboard... subsystems) {

  for(ImplementDashboard s: subsystems) {
  // Organize subsystems to their respective updateDashboard use types
  if(s.getDashboardUses() == DashboardUses.SHORT_INTERVAL) dashboardShortUpdates.add(s);
  if(s.getDashboardUses() == DashboardUses.LONG_INTERVAL) dashboardShortUpdates.add(s);
  // Run dashboard initialize functions at construction (which is at RobotContainer construction therefore robotInit)
  s.initDashboard();
  }

  startCommandLogs();
}

public void startCommandLogs() {
  CommandScheduler.getInstance().onCommandInterrupt((Command action) -> 
  SmartDashboard.putString(action.getSubsystem() + "Command Log", action.getName() + " Command has been interrupted")
  );
}

  @Override
  public void periodic() {
    double time = Timer.getFPGATimestamp();
    if (kEnableDashBoard) {
        if ((time - m_shortOldTime) > kShortInterval) {
            m_shortOldTime = time;
            for(ImplementDashboard s: dashboardShortUpdates) s.updateDashboard();
        }
        if ((time - m_longOldTime) > kLongInterval) {
            // Thing that should be updated every LONG_DELAY
            m_longOldTime = time;
            for(ImplementDashboard s: dashboardShortUpdates) s.updateDashboard();
            for(ImplementDashboard s: dashboardLongUpdates) s.updateDashboard();
        }
  }
  
}
}