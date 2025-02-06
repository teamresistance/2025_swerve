// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhysicalReefInterfaceSubsystem extends SubsystemBase {
  /** Creates a new PhysicalReefSubsystem. */
  public PhysicalReefInterfaceSubsystem() {}

  private int level;
  private int pos;
  private boolean rl = false; // Left is false, right is true

  public void ChooseReef(int level, int pos, int rl) {
    if (rl == 1) {
      this.rl = !this.rl;
    }
    this.level = level;
    this.pos = pos;
    // Drive functiosn to rotate will be added later maybe
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
