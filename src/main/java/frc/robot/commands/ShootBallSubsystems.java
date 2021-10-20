package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootBallSubsystems extends CommandBase {
  /** Creates a new ShootBallSubsystems. */
  private Shooter shooter;
  private Storage storage;
  private Feeder feeder;

  //TODO RPM VALUE
  public int RPM = 6000;

  public ShootBallSubsystems(Shooter shooter, Feeder feeder, Storage storage) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.storage = storage;
    addRequirements(shooter,feeder,storage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setRPM(RPM);
    if(shooter.isAtRPM(RPM)){
      feeder.runForward();
      storage.bothForward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.isAtRPM(RPM)){
      return false;
    }
    else return true;
  }
}
