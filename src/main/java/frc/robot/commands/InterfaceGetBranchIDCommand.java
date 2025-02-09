// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.InterfaceSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class InterfaceGetBranchIDCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final InterfaceSubsystem m_subsystem;
  private final String id;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public InterfaceGetBranchIDCommand(InterfaceSubsystem subsystem, String id) {
    m_subsystem = subsystem;
    this.id = id;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("Storing Branch ID: ");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.storeBranchID(id);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Stored Branch ID!");
    return true;
  }
}
