// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.HardwareConstants;

import edu.wpi.first.wpilibj.Solenoid;


public class ArmSubsystem extends SubsystemBase {
  public Solenoid wristRotator = new Solenoid(
    HardwareConstants.pneumaticsModuleType, 
    HardwareConstants.kSolenoid_wristRotator_portNumber
    );
  
  public Solenoid armLifter = new Solenoid(
    HardwareConstants.pneumaticsModuleType, 
    HardwareConstants.kSolenoid_armLifter_portNumber
    );

  public boolean hasCoral = false;
  public boolean inScoringPosition = false;

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return the opposite of the value of said boolean state.
   */
  public void getCoral() {
    hasCoral = true;
  }

  public void rotateArmAxis() {
    armLifter.setPulseDuration(
      HardwareConstants.kArmLifterPulseDurationSeconds
    );
    armLifter.set(true);
  }

  public void rotateArmWrist() {
    wristRotator.setPulseDuration(
      HardwareConstants.kWristRotatorPulseDurationSeconds
    );
    wristRotator.set(true);
  }
  public void getInScoringPosition() {
    rotateArmAxis();
    rotateArmWrist();
    inScoringPosition = true;
  }

  public void score() {
    try {
      Thread.sleep(RobotConstants.kScoreTimeoutMilliseconds);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }

    hasCoral = false;
  }

  public void getBackToReceivingPosition() {
    wristRotator.set(false);
    armLifter.set(false);

    inScoringPosition = false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Coral?", hasCoral);
    SmartDashboard.putBoolean("In Scoring Position?", inScoringPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
