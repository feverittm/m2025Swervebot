package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class GoToTag extends Command {

  private final Drivebase drivebase;
  private final Supplier<Translation2d> robotToTag;
  private final Double radius;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController TurningController;

  public GoToTag(Drivebase drivebase, Supplier<Translation2d> robotToTag, Double radius) {
    this.drivebase = drivebase;
    this.robotToTag = robotToTag;
    this.radius = radius;
    this.xController = new PIDController(0.25, 0, 0);
    this.yController = new PIDController(0.25, 0, 0);
    this.TurningController = new PIDController(0.15, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = this.robotToTag.get();
    double xSpeed = -xController.calculate(translation.getX());
    double ySpeed = -yController.calculate(translation.getY()); 
    double rotationSpeed = -TurningController.calculate(translation.getAngle().getDegrees());

    SmartDashboard.putBoolean("GOTOTAG RUNNING", true);

    SmartDashboard.putNumber("xSpeed Tag", xSpeed);
    SmartDashboard.putNumber("ySpeed Tag", ySpeed);
    SmartDashboard.putNumber("turnSpeed Tag", rotationSpeed);

    SmartDashboard.putNumber("Robot to tag Translation x", translation.getX());
    SmartDashboard.putNumber("Robot to tag Translation y", translation.getY());
    SmartDashboard.putNumber("Robot to tag Rotation Yaw", translation.getAngle().getDegrees());

    drivebase.defaultDrive(xSpeed, ySpeed, rotationSpeed);
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
