// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FlipperBackToReceivingPositionCommand;
import frc.robot.commands.FlipperScoringCommand;
import frc.robot.subsystems.FlipperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCommandGroup extends SequentialCommandGroup {
  /** Creates a new Level1CommandGroup. */
  public ScoreCommandGroup(FlipperSubsystem subsystem) {
    addCommands(
        new FlipperScoringCommand(subsystem), new FlipperBackToReceivingPositionCommand(subsystem));
  }
}
