
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;



public class Shooter extends SubsystemBase{
    public static WPI_TalonSRX masterMotor;
    public static WPI_TalonSRX slaveMotor;
    public Encoder shooterEncoder;

    private double kP = 0.000109; //proportional
    private double kI = 0; //integral
    private double kD = 0.0; //derivative
    //private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Shooter.kS, Constants.Shooter.kV, Constants.Shooter.kA);
    private PIDController pid = new PIDController(kP,kI,kD);

    private double ENCODER_EDGES_PER_REV = 4096 / 4.;
    private double encoderConstant = (1 / ENCODER_EDGES_PER_REV);
    

    public Shooter(){
        masterMotor = new WPI_TalonSRX(4);
        slaveMotor = new WPI_TalonSRX(9);
        shooterEncoder = new Encoder(0, 1, false);
        shooterEncoder.setDistancePerPulse(1/(4096/4.0));
        
        slaveMotor.setInverted(false);
        masterMotor.setInverted(false);
        //slaveMotor.follow(masterMotor);

        //shooterEncoder.setDistancePerPulse(encoderConstant);
        pid.setTolerance(100);
    }
    public void setShooter(double percentage) {
      masterMotor.set(percentage);
      slaveMotor.set(percentage);
    }


    public double getRPM(){
        return shooterEncoder.getRate() * 60.;
    }

    public void stop(){
        setShooter(0);
    }
    public boolean isAtRPM(int RPM){
        if(Math.abs(getRPM() - RPM) <= 100){
            return true;
        }
        return false;
    }

    // ? decide whether we should use PIDcommand or setRPM in the subsystem
    public void setRPM(int rpm){  
        setShooter(
          MathUtil.clamp(
            pid.calculate(getRPM(), rpm),
            -1,1)
            );        
    }

    public void runBackwards(){
        setShooter(-0.2);
    }
    
}
