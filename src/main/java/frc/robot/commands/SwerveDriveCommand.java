// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDriveCommand extends CommandBase {
  private final Joystick joystick;
  private final Swerve swerveSubsystem;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  // TODO not using them currently, try out and see if you want to keep them for comp
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  
  /** Creates a new SwerveDriveCommand. */
  public SwerveDriveCommand(Swerve sw, Joystick joystick) {
    this.swerveSubsystem = sw;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final var xSpeed = xSpeedLimiter.calculate(joystick.getRawAxis(1) * Constants.Swerve.kMaxSpeed);
    
    final var ySpeed = ySpeedLimiter.calculate(joystick.getRawAxis(0) * Constants.Swerve.kMaxSpeed );
    final var rot = rotLimiter.calculate( joystick.getRawAxis(2) * Constants.Swerve.kMaxAngularSpeed) ;
    boolean fieldRelative = true;
    swerveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
