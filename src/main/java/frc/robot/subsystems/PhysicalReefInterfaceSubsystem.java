// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhysicalReefInterfaceSubsystem extends SubsystemBase {
  /** Creates a new PhysicalReefSubsystem. */
  public PhysicalReefInterfaceSubsystem() {}

  private int level;
  private int pos;
  private boolean rl = false; // Left is false, right is true

  public void ChooseReef() {
    SmartDashboard.putString(
        "Last Selection", "Level = " + level + ", Position = " + pos + ", L/R = " + rl);
    level = 0;
    pos = 0;
    rl = false;
  }

  // Add code to make the robot go to X location

  public void ChooseVars(int level, int pos, int rl) {
    if (rl == 1) {
      this.rl = !this.rl;
    }
    if (level > -1) {
      this.level = level;
    }
    if (pos > -1) {
      this.pos = pos;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Level", level);
    SmartDashboard.putNumber("Position", pos);
    SmartDashboard.putBoolean("Side (right=true)", rl);
    // This method will be called once per scheduler run
  }
}
