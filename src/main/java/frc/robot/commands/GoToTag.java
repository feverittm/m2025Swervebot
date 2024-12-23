package frc.robot.commands;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Camera;

public class GoToTag extends Command {

  private final Drivebase drivebase;
  private final Camera frontCamera;
  private final Double radius;
  private Command currentPath;

  public GoToTag(Drivebase drivebase, Camera frontCamera, Double radius) {
    this.drivebase = drivebase;
    this.frontCamera = frontCamera;
    this.radius = radius;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (frontCamera.get_tag_Yaw() != 0)
    {
    double theta = frontCamera.robot_to_tag(drivebase).getAngle().getRadians();
    Transform2d tagOffset = new Transform2d(this.radius*Math.cos(theta), this.radius*Math.sin(theta), new Rotation2d(theta));
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses
    (
      this.drivebase.getPose(),
      this.frontCamera.get_tag_pose2d().plus(tagOffset)
    );
    PathPlannerPath path = new PathPlannerPath(
        bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
        new GoalEndState(0.0, new Rotation2d(theta))); // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    this.currentPath = AutoBuilder.followPath(path);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    this.currentPath.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
