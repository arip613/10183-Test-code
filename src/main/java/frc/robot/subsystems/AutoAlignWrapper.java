import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoAlignWrapper extends SequentialCommandGroup {
    public AutoAlignWrapper(SwerveSubsystem drivebase, boolean alignToReefTagRelative) {
        addCommands(
            // Switch to robot-relative mode
            new InstantCommand(() -> drivebase.setRobotRelative(true)),
            
            // Execute the AlignToReefTagRelative command
            new AlignToReefTagRelative(alignToReefTagRelative, drivebase).withTimeout(3),
            
            // Revert to field-relative mode
            new InstantCommand(() -> drivebase.setRobotRelative(false))
        );
    }
}
