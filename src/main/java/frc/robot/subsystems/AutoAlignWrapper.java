import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoAlignWrapper extends SequentialCommandGroup {
    public AutoAlignWrapper(SwerveSubsystem drivebase, boolean alignToReefTagRelative) {
        addCommands(
            // Enable heading correction
            new InstantCommand(() -> drivebase.setHeadingCorrection(true)),

            // Perform the auto-align
            new AlignToReefTagRelative(alignToReefTagRelative, drivebase).withTimeout(3),

            // Disable heading correction (if not needed after alignment)
            new InstantCommand(() -> drivebase.setHeadingCorrection(false))
        );
    }
}
