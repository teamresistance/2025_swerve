// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorRaiseFirstStageCommand;
import frc.robot.commands.ElevatorRaiseSecondStageCommand;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorHighestCommandGroup extends SequentialCommandGroup {
  /** Creates a new ElevatorLowestCommandGroup. */
  public ElevatorHighestCommandGroup(ElevatorSubsystem subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorRaiseFirstStageCommand(subsystem),
      new ElevatorRaiseSecondStageCommand(subsystem)
    );
  }
}
