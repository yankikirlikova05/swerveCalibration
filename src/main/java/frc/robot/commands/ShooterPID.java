// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterPID extends CommandBase {
  int rpm;
  /** Creates a new ShooterPID. */
  private Shooter shooter;
  private double scaleUp = 0;

  public ShooterPID(Shooter shooter1, int rpm) {
    this.rpm = rpm;
    this.shooter = shooter1;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scaleUp = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.setRPM(6000);
    //shooter.setShooter(percentage*scaleUp);
    shooter.setRPM(rpm);

   // if (scaleUp <= 1) scaleUp += 0.05;
    // shooter.setShooter(0.5);
    SmartDashboard.putNumber("Shooter RPM", shooter.getRPM());
    SmartDashboard.putNumber("Feedforward Value",shooter.feedforward.calculate(6000));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isAtRPM(rpm);
  }
}
