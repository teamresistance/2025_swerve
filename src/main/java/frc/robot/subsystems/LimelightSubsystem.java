package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.FieldConstants;

public class LimelightSubsystem extends SubsystemBase {
    
    public LimelightSubsystem() {}

    @Override
    public void periodic() {
        double tx = LimelightHelpers.getTX(RobotConstants.limelightName);
        double ty = LimelightHelpers.getTY(RobotConstants.limelightName);
        double ta = LimelightHelpers.getTA(RobotConstants.limelightName);

        double perceivedBranchWidthPixels = 
        LimelightHelpers.getT2DArray(
            RobotConstants.limelightName)[13];

        double forwardDistanceToBranchInches = 
            (RobotConstants.kLimelightWindowResolutionWidthPixels)
            * (FieldConstants.kReefBranchWidthInches)
            / (perceivedBranchWidthPixels
            * Math.sin(
                Math.toRadians(
                    RobotConstants.kLimelightHorizontalFOVdegrees
                    / 2))
            * 2);

        double horizontalOffsetToBranchInches = 
            forwardDistanceToBranchInches
            * Math.tan(
                Math.toRadians(
                    tx
                ));

        SmartDashboard.putNumber("X-Offset in degrees", tx);
        SmartDashboard.putNumber("Y-Offset in degrees", ty);
        SmartDashboard.putNumber("Area of target in %", ta);

    }

    @Override
    public void simulationPeriodic() {
        double tx = LimelightHelpers.getTX(RobotConstants.limelightName);
        double ty = LimelightHelpers.getTY(RobotConstants.limelightName);
        double ta = LimelightHelpers.getTA(RobotConstants.limelightName);

        SmartDashboard.putNumber("X-Offset in degrees", tx);
        SmartDashboard.putNumber("Y-Offset in degrees", ty);
        SmartDashboard.putNumber("Area of target in %", ta);
    }
}