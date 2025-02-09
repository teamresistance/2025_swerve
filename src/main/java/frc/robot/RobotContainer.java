// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//  ooooooooo.              .o8                     .     .oooooo.                             .              o8o                                 
//  `888   `Y88.           "888                   .o8    d8P'  `Y8b                          .o8              `"'                                 
//   888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo 888           .ooooo.  ooo. .oo.   .o888oo  .oooo.   oooo  ooo. .oo.    .ooooo.  oooo d8b 
//   888ooo88P'  d88' `88b  d88' `88b d88' `88b   888   888          d88' `88b `888P"Y88b    888   `P  )88b  `888  `888P"Y88b  d88' `88b `888""8P 
//   888`88b.    888   888  888   888 888   888   888   888          888   888  888   888    888    .oP"888   888   888   888  888ooo888  888     
//   888  `88b.  888   888  888   888 888   888   888 . `88b    ooo  888   888  888   888    888 . d8(  888   888   888   888  888    .o  888     
//  o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888"  `Y8bood8P'  `Y8bod8P' o888o o888o   "888" `Y888""8o o888o o888o o888o `Y8bod8P' d888b    

// Constants
import frc.robot.Constants.OperatorConstants;

// Operator Input
import frc.robot.OperatorInput;


//  .oooooo.                                                                             .o8           
// d8P'  `Y8b                                                                           "888           
//888           .ooooo.  ooo. .oo.  .oo.   ooo. .oo.  .oo.    .oooo.   ooo. .oo.    .oooo888   .oooo.o 
//888          d88' `88b `888P"Y88bP"Y88b  `888P"Y88bP"Y88b  `P  )88b  `888P"Y88b  d88' `888  d88(  "8 
//888          888   888  888   888   888   888   888   888   .oP"888   888   888  888   888  `"Y88b.  
//`88b    ooo  888   888  888   888   888   888   888   888  d8(  888   888   888  888   888  o.  )88b 
// `Y8bood8P'  `Y8bod8P' o888o o888o o888o o888o o888o o888o `Y888""8o o888o o888o `Y8bod88P" 8""888P' 

// Command Groups
import frc.robot.commandgroups.Level2CommandGroup;
import frc.robot.commandgroups.Level3CommandGroup;
import frc.robot.commandgroups.Level4CommandGroup;

// Elevator Commands
// NONE YET!

// Arm Commands
// NONE YET!

// Interface Commands
import frc.robot.commands.InterfaceStoreBranchesCommand;
import frc.robot.commands.InterfaceToggleLeftRightCommand;
import frc.robot.commands.InterfaceGetBranchIDCommand;
import frc.robot.commands.InterfaceGetBranchLevelCommand;

//   .oooooo..o              .o8                                         .                                        
//  d8P'    `Y8             "888                                       .o8                                        
//  Y88bo.      oooo  oooo   888oooo.   .oooo.o oooo    ooo  .oooo.o .o888oo  .ooooo.  ooo. .oo.  .oo.    .oooo.o 
//   `"Y8888o.  `888  `888   d88' `88b d88(  "8  `88.  .8'  d88(  "8   888   d88' `88b `888P"Y88bP"Y88b  d88(  "8 
//       `"Y88b  888   888   888   888 `"Y88b.    `88..8'   `"Y88b.    888   888ooo888  888   888   888  `"Y88b.  
//  oo     .d8P  888   888   888   888 o.  )88b    `888'    o.  )88b   888 . 888    .o  888   888   888  o.  )88b 
//  8""88888P'   `V88V"V8P'  `Y8bod8P' 8""888P'     .8'     8""888P'   "888" `Y8bod8P' o888o o888o o888o 8""888P' 
//                                              .o..P'                                                            
//                                              `Y8P'                                                             

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.InterfaceSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//    .oooooo.   ooooo 
//   d8P'  `Y8b  `888' 
//  888      888  888  
//  888      888  888  
//  888      888  888  
//  `88b    d88'  888  
//   `Y8bood8P'  o888o 
//                    

// Necessary stuff
// import edu.wpi.first.wpilibj2.command.Command;  <------------- Uncomment this if you are using a command without a group
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  final double DEADZONE = 0.1;

  // Create the target Transform2d (Translation and Rotation)
  Translation2d targetTranslation = new Translation2d(15, 4); // X = 14, Y = 4
  Rotation2d targetRotation = new Rotation2d(0.0); // No rotation
  Transform2d targetTransform = new Transform2d(targetTranslation, targetRotation);

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
  }
    private final OperatorInput m_operatorInput = new OperatorInput(1);

    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final FlipperSubsystem m_flipperSubsystem = new FlipperSubsystem();
    private final InterfaceSubsystem m_interfaceSubsystem = new InterfaceSubsystem();
    private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

    private final Level2CommandGroup m_level2CommandGroup = new Level2CommandGroup(m_flipperSubsystem);
    private final Level3CommandGroup m_level3CommandGroup = new Level3CommandGroup(m_elevatorSubsystem, m_flipperSubsystem);
    private final Level4CommandGroup m_level4CommandGroup = new Level4CommandGroup(m_elevatorSubsystem, m_flipperSubsystem);

    private final InterfaceStoreBranchesCommand m_interfaceStoreBranchesCommand = new InterfaceStoreBranchesCommand(m_interfaceSubsystem);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -applyDeadband(controller.getLeftY()),
            () -> -applyDeadband(controller.getLeftX()),
            () -> -applyDeadband(controller.getRightX())));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -applyDeadband(controller.getLeftY()),
                () -> -applyDeadband(controller.getLeftX()),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller.leftBumper().whileTrue(DriveCommands.goToTransform(drive, targetTransform));

    // **Left Trigger - Go to AprilTag Position A**
    controller
        .leftTrigger()
        .whileTrue(DriveCommands.goTo2DPos(drive, 0.0, 1.0, 0.0)); // Example values

    // **Right Trigger - Go to AprilTag Position B**
    controller
        .rightTrigger()
        .whileTrue(DriveCommands.goTo2DPos(drive, 1.0, 2.0, 0.0)); // Example values
    
        if (m_operatorInput.jsType != 1) {
            m_operatorInput.lvl2Button.onTrue(m_level2CommandGroup);
            m_operatorInput.lvl3Button.onTrue(m_level3CommandGroup);
            m_operatorInput.lvl4Button.onTrue(m_level4CommandGroup);
            m_operatorInput.selectBranchAndAddButton.onTrue(m_interfaceStoreBranchesCommand);
          } else {
            m_operatorInput.buttonA.onTrue(
              new InterfaceGetBranchIDCommand(m_interfaceSubsystem, "A")
            );
            m_operatorInput.buttonB.onTrue(
              new InterfaceGetBranchIDCommand(m_interfaceSubsystem, "B")
            );
            m_operatorInput.buttonC.onTrue(
              new InterfaceGetBranchIDCommand(m_interfaceSubsystem, "C")
            );
            m_operatorInput.buttonD.onTrue(
              new InterfaceGetBranchIDCommand(m_interfaceSubsystem, "D")
            );
            m_operatorInput.buttonE.onTrue(
              new InterfaceGetBranchIDCommand(m_interfaceSubsystem, "E")
            );
            m_operatorInput.buttonF.onTrue(
              new InterfaceGetBranchIDCommand(m_interfaceSubsystem, "F")
            );
      
            m_operatorInput.buttonRL.onTrue(
              new InterfaceToggleLeftRightCommand(m_interfaceSubsystem)
            );
      
            m_operatorInput.button4.onTrue(
              new InterfaceGetBranchLevelCommand(m_interfaceSubsystem, 4)
            );
            m_operatorInput.button3.onTrue(
              new InterfaceGetBranchLevelCommand(m_interfaceSubsystem, 3)
            );
            m_operatorInput.button2_1.onTrue(
              new InterfaceGetBranchLevelCommand(m_interfaceSubsystem, 2)
            );
          }
        }
    //		controller
    //			.b()
    //			.onTrue(
    //				Commands.runOnce(
    //						() ->
    //							drive.setPose(
    //								new Pose2d(drive.getPose().getTranslation(), new Rotation2d())), // reset gyro
    //						drive)
    //					.ignoringDisable(true));
    //    controller
    //        .a()
    //        .whileTrue(
    //            Commands.startEnd(
    //                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop,
    // flywheel));
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // Utility function to apply deadband
  private double applyDeadband(double value) {
    return (Math.abs(value) > DEADZONE) ? value : 0.0;
  }
}
