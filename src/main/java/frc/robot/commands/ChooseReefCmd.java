// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhysicalReefInterfaceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChooseReefCmd extends Command {
  /** Creates a new ChooseReefCmd. */
  private PhysicalReefInterfaceSubsystem subsystem;

  private int level;
  private int pos;
  private int rl;
  private boolean exec;

  public ChooseReefCmd(
      PhysicalReefInterfaceSubsystem subsystem, int level, int pos, int rl, boolean exec) {
    this.subsystem = subsystem;
    if (level != -1) {
      this.level = level;
    }
    if (level != -1) {
      this.pos = pos;
    }
    if (level != -1) {
      this.rl = rl;
    }
    this.exec = exec;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (exec) {
      subsystem.ChooseReef(level, pos, rl);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
