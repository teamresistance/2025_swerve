// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class ElevatorSubsystem extends SubsystemBase {
  Solenoid ElevatorPusher1= new Solenoid(Constants.SolenoidModuleType, Constants.ElevatorSolenoid1Channel);
  Solenoid ElevatorPusher2= new Solenoid(Constants.SolenoidModuleType, Constants.ElevatorSolenoid2Channel);
  public boolean firstStageSolenoidUp = false;
  public boolean secondStageSolenoidUp = false;
  public boolean firstStageSolenoidDown=false;
  public boolean secondStageSolenoidDown=false;
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return the opposite of the value of said boolean state.
   */
  public void raiseFirstStage() {
    ElevatorPusher1.setPulseDuration(1.0); // Assuming x is 1.0, replace with the correct value
    ElevatorPusher1.set(true);
    firstStageSolenoidUp = true;

  }
  public void raiseSecondStage() {
    raiseFirstStage();
    ElevatorPusher2.setPulseDuration(1.0); // Assuming x is 1.0, replace with the correct value
    secondStageSolenoidUp=true;
  }
  public void lowerSecondStage() {
    ElevatorPusher2.set(false);
    firstStageSolenoidDown=true;
    lowerFirstStage();
  }
  public void lowerFirstStage() {
    ElevatorPusher1.set(false);
    secondStageSolenoidDown=true;
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("First Stage Up?", firstStageSolenoidUp);
    SmartDashboard.putBoolean("Second Stage Up?", secondStageSolenoidUp);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
