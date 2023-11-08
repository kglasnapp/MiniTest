// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.utilities.Util.logf;

public class CoralSubsystem extends SubsystemBase {
  public boolean found = false;
  public double x;
  public double y;
  public double percent;
  public double area;
  public String type;
  private String lookForType = "";

  NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
  NetworkTableEntry xMidE = table.getEntry("xMid");
  NetworkTableEntry yMidE = table.getEntry("yMid");
  NetworkTableEntry areaE = table.getEntry("area");
  NetworkTableEntry typeE = table.getEntry("type");
  NetworkTableEntry percentE = table.getEntry("percent");

  /** Creates a new ReplaceMeSubs`ystem. */
  public CoralSubsystem() {
    logf("Start of Coral Subsystem\n");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Make sure that you declare this subsystem in RobotContainer.java
    percent = percentE.getDouble(0.0);
    type = typeE.getString("None");
    if (type == lookForType & percent > 55) {
      x = xMidE.getDouble(0.0);
      y = yMidE.getDouble(0.0);
      percent = percentE.getDouble(0.0);
      area = areaE.getDouble(0.0);
      if (Robot.count % 10 == 0) {
        logf("Coral x:%.0f y:%0.f per:%.1f area:%.0f\n", x, y, percent, area);
      }
    }
  }

  public void setLookForType(String type) {
    lookForType = type;
  }
}