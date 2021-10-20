// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */
  public Shooter shooter;
  public Swerve swerve;
  public LEDSubsystem led;

  private PhotonCamera camera = new PhotonCamera("photonvision");

  double yaw;
  double pitch;
  double skew;
  Transform2d pose;
  Translation2d  translation;

  double range;

  double targetAngle;
  TurnToAngle turnToAngle;

  //TODO ! DISABLE SWERVEDRIVE COMMAND 
  public AutoShoot(Shooter shooter, Swerve swerve,TurnToAngle turnToAngle) {
    this.turnToAngle = turnToAngle;
    this.shooter = shooter;
    this.swerve = swerve;

    camera.setDriverMode(false);
    
    addRequirements(led,shooter,swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.turnOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  //TODO goTo 
  @Override
  public void execute() {
    var result = camera.getLatestResult();
    if(result.hasTargets()){
      PhotonTrackedTarget target = result.getBestTarget();

      yaw = target.getYaw();
      pitch = target.getPitch();
      skew = target.getSkew();
      pose = target.getCameraToTarget();

      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Pitch", pitch);
      
       range = PhotonUtils.calculateDistanceToTargetMeters(
       Constants.CAMERA_HEIGHT_METERS, 
       Constants.TARGET_HEIGHT_METERS,
       Constants.CAMERA_PITCH_RADIANS,
       Units.degreesToRadians(result.getBestTarget().getPitch()));

       translation = PhotonUtils.estimateCameraToTargetTranslation(range, Rotation2d.fromDegrees(-target.getYaw()));

       targetAngle = Math.asin(translation.getY()/range);

      /* EKİNİN METHODUYLA İLGİLİ
      Pose2d goal = new Pose2d(translation,
        new Rotation2d(result.getBestTarget().getPitch(),result.getBestTarget().getYaw()));
      
      boolean poseChanged = swerve.goTo(goal);
      
      if(poseChanged) shooter.setRPM(6000);
       /*Pose2D robotPose = PhotonUtils.estimateFieldToRobot(
        Constants.CAMERA_HEIGHT_METERS, Constants.TARGET_HEIGHT_METERS, 
        Constants.CAMERA_PITCH_RADIANS, kTargetPitch, 
        Rotation2d.fromDegrees(-target.getYaw()), swerve.getHeading(), 
        targetPose, cameraToRobot);*/
    }
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
