package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignToReefTagRelative extends Command {
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private SwerveSubsystem drivebase;
    private double tagID = -1; // Invalid tag ID as default

    public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
        xController = new PIDController(VisionConstants.X_REEF_ALIGNMENT_P, 0.0, 0.0);  // Vertical movement
        yController = new PIDController(VisionConstants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);  // Horizontal movement
        rotController = new PIDController(VisionConstants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0);  // Rotation
        this.isRightScore = isRightScore;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        // Set PID Controller Setpoints and Tolerances
        rotController.setSetpoint(VisionConstants.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(VisionConstants.ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(VisionConstants.X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(VisionConstants.X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(isRightScore ? VisionConstants.Y_SETPOINT_REEF_ALIGNMENT : -VisionConstants.Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(VisionConstants.Y_TOLERANCE_REEF_ALIGNMENT);

        // Get initial tag ID and validate it
        double detectedTagID = LimelightHelpers.getFiducialID("");
        if (detectedTagID >= 0) {
            tagID = detectedTagID;
        } else {
            SmartDashboard.putString("AlignToReefTagRelative Error", "No valid tag ID detected during initialization.");
        }
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
            this.dontSeeTagTimer.reset(); // Reset timer when tag is visible

            double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
            if (positions.length < 5) {
                SmartDashboard.putString("AlignToReefTagRelative Error", "Invalid position array from Limelight.");
                return;
            }

            //// debugging stuff
            SmartDashboard.putNumber("x Position", positions[2]);
            SmartDashboard.putNumber("y Position", positions[0]);
            SmartDashboard.putNumber("Rotation Position", positions[4]);

            // pid math
            double xSpeed = xController.calculate(positions[2]);
            double ySpeed = -yController.calculate(positions[0]);
            double rotValue = -rotController.calculate(positions[4]);

            // debugging stuff
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            SmartDashboard.putNumber("rotValue", rotValue);

            // drive
            drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

            // reset stop timer if not at setpoints
            if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            // tag not visible logic
            drivebase.drive(new Translation2d(), 0, false);
            SmartDashboard.putString("AlignToReefTagRelative Warning", "Tag not visible.");
        }

        // Debugging outputs for timer
        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(new Translation2d(), 0, false); // Stop the robot
    }

    @Override
    public boolean isFinished() {
        // Return true if no tag is seen for a while or if the pose is valid for the required time
        return this.dontSeeTagTimer.hasElapsed(VisionConstants.DONT_SEE_TAG_WAIT_TIME) ||
               stopTimer.hasElapsed(VisionConstants.POSE_VALIDATION_TIME);
    }
}
