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

import edu.wpi.first.wpilibj.RobotBase;

//Hardware imports
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public static class OperatorConstants {
    public static final int klvl2Button_3Joysticks_ID = 3;
    public static final int klvl3Button_3Joysticks_ID = 4;
    public static final int klvl4Button_3Joysticks_ID = 6;
    public static final int kSelectBranchAndAddButton_3Joysticks_ID = 1;

    public static final int klvl2Button_Neo_ID = 0;
    public static final int klvl3Button_Neo_ID = 0;
    public static final int klvl4Button_Neo_ID = 0;
    public static final int kSelectBranchAndAddButton_Neo_ID = 0;

    public static final int klvl2Button_Kybd1_ID = 0;
    public static final int klvl3Button_Kybd1_ID = 0;
    public static final int klvl4Button_Kybd1_ID = 0;
    public static final int kSelectBranchAndAddButton_Kybd1_ID = 0;

  }

  public static class RobotConstants {
    public static final int kScoreTimeoutMilliseconds = 3000; //Milliseconds
    public static final int kGripperDelayMilliseconds = 200; //Milliseconds

    public static final String limelightName = "limelight";
    public static final double kLimelightWindowResolutionWidthPixels = 960;
    public static final double kLimelightHorizontalFOVdegrees = 62.5;
  }

  public static class HardwareConstants {
    public static final PneumaticsModuleType pneumaticsModuleType = PneumaticsModuleType.REVPH;
    
    //Arm
    public static final int kSolenoid_wristRotator_portNumber = 2;
    public static final int kWristRotatorPulseDurationSeconds = 1;
    public static final int kSolenoid_armLifter_portNumber = 0;
    public static final int kArmLifterPulseDurationSeconds = 1;

    //Elevator
    public static final int kSolenoid_firstStage_portNumber = 1;
    public static final int kFirstStagePulseDurationSeconds = 1;
    public static final int kSolenoid_secondStage_portNumber = 5;
    public static final int kSecondStagePulseDurationSeconds = 1;
  }

  public static class FieldConstants {
    public static final double kReefBranchWidthInches = 1;
  }

}