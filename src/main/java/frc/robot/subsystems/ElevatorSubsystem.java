package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX leader;
    private TalonFX follower;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private int targetPosition;
    private MotionMagicExpoVoltage m_request;

    public ElevatorSubsystem() {

        leader = new TalonFX(Constants.ElevatorConstants.leaderID, "rio");
        follower = new TalonFX(Constants.ElevatorConstants.followerID, "rio");

        //.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        targetPosition = 0;

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // in init function
        talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.4; 
        slot0Configs.kV = 0.001; 
        slot0Configs.kA = 0;
        slot0Configs.kG = 0.5; 
        slot0Configs.kP = 7;
        slot0Configs.kI = 0; 
        slot0Configs.kD = 0.001;
       

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 70; // Target acceleration of 100 rps/s
        motionMagicConfigs.MotionMagicJerk = 600; // Target jerk of 100 rps/s/s
        motionMagicConfigs.MotionMagicCruiseVelocity = 45;
        motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0.1;
        
         // Use a slower kA of 0.1 V/(rps/s)
        

        leader.getConfigurator().apply(talonFXConfigs);

        m_request = new MotionMagicExpoVoltage(0);

        // set target position to 100 rotations
        
        follower.setControl(new Follower(leader.getDeviceID(), true));
        
        

        
    }
    public void resetEncoderElevator() {
        leader.setPosition(0);
      }

    public BooleanSupplier isAtTarget(){
        return (BooleanSupplier) () -> { return this.leader.getClosedLoopReference().getValueAsDouble() < 40;
        };
         
    }

    public double convertEncoderToRotation(double encoderValue)
    {
        return encoderValue / 2048;
    }

    public BooleanSupplier finishedMotionMagic()
    {
        return (BooleanSupplier) () -> { return (leader.getPosition().getValueAsDouble() >= (m_request.Position - 0.25) && leader.getPosition().getValueAsDouble() <= (m_request.Position + 0.25));
        };
    }

    // public Command RasiseWhileAligning(){
    // return
    // run(null).onlyWhile(RobotContainer.isNotAligned()).finallyDo(this::raiseElevatorTop);
    // }


    //FIGURE THIS OUT LATER ONCE WE KNOW THE ENCODER VALUES
    // public BooleanSupplier L4AchieveHorizontalElevatorClearance() {
    //     return (BooleanSupplier) () -> {
    //         return this.leader.getEncoder().getPosition() < 40;
    //     };
    // }

    // public BooleanSupplier L3AchieveHorizontalElevatorClearance() {
    //     return (BooleanSupplier) () -> {
    //         return this.leader.getEncoder().getPosition() < 23;
    //     };
    // }

    // public BooleanSupplier L2AchieveHorizontalElevatorClearance() {
    //     return (BooleanSupplier) () -> {
    //         return this.leader.getEncoder().getPosition() < 8;
    //     };
    // }

    public Command raiseElevatorL4()
    {
        // return this.runOnce(
        //     () -> leader.setControl(m_request.withPosition(19))
        // );

        // if (DriveSubsystem.isCloseEnough().getAsBoolean())
        //     return run(() -> leader.setControl(m_request.withPosition(19.1))).until(finishedMotionMagic());//19.1
        // else
        //     return runOnce(() -> leader.setControl(m_request.withPosition(1.8)));
        return run(() -> leader.setControl(m_request.withPosition(1.8))).until(() -> DriveSubsystem.isCloseEnough().getAsBoolean())
        .andThen(run(() -> leader.setControl(m_request.withPosition(19.1))).until(finishedMotionMagic()));

        // (() -> leader.setControl(m_request.withPosition(19.1)), () -> leader.setControl(m_request.withPosition(19.1))).onlyWhile(DriveSubsystem.isCloseEnough()).until(finishedMotionMagic());
    }
    public Command raiseElevatorL3()
    {
        // return this.runOnce(
        //     () -> leader.setControl(m_request.withPosition(11.25))
        // );

        // return startEnd(() -> leader.setControl(m_request.withPosition(11.25)), () -> leader.setControl(m_request.withPosition(11.25))).onlyWhile(DriveSubsystem.isCloseEnough()).until(finishedMotionMagic());
         return run(() -> leader.setControl(m_request.withPosition(1.8))).until(() -> DriveSubsystem.isCloseEnough().getAsBoolean())
        .andThen(run(() -> leader.setControl(m_request.withPosition(12.25))).until(finishedMotionMagic()));
    }
    public Command raiseElevatorL2()
    {
        // return this.runOnce(
        //     () -> leader.setControl(m_request.withPosition(7.5))
        // );

        // return startEnd(() -> leader.setControl(m_request.withPosition(7.5)), () -> leader.setControl(m_request.withPosition(7.5))).onlyWhile(DriveSubsystem.isCloseEnough()).until(finishedMotionMagic());
        return run(() -> leader.setControl(m_request.withPosition(1.8))).until(() -> DriveSubsystem.isCloseEnough().getAsBoolean())
        .andThen(run(() -> leader.setControl(m_request.withPosition(8))).until(finishedMotionMagic()));
    }
    public Command raiseElevatorNet()
    {
        // return this.runOnce(
        //     () -> leader.setControl(m_request.withPosition(12))
        // );
    
        // return startEnd(() -> leader.setControl(m_request.withPosition(12)), () -> leader.setControl(m_request.withPosition(12))).until(finishedMotionMagic());
        return runOnce(() -> leader.setControl(m_request.withPosition(21))).until(finishedMotionMagic());
    }
    public Command raiseElevatorCam()
    {
        // return this.runOnce(
        //     () -> leader.setControl(m_request.withPosition(19))
        // );

        return startEnd(() -> leader.setControl(m_request.withPosition(1.8)), () -> leader.setControl(m_request.withPosition(1.8))).until(finishedMotionMagic());
    }
    public Command raiseElevatorCamA()
    {
        // return this.runOnce(
        //     () -> leader.setControl(m_request.withPosition(19))
        // );

        return runOnce(()->leader.setControl(m_request.withPosition(1.8)));
    }


    public Command lowerElevator()
    {
        return this.runOnce(
            () -> leader.setControl(m_request.withPosition(0))
        );
    }
    public Command raiseHighAlgaePrep()
    {
        // return this.runOnce(
        //     () -> leader.setControl(m_request.withPosition(12))
        // );
    
        // return startEnd(() -> leader.setControl(m_request.withPosition(12)), () -> leader.setControl(m_request.withPosition(12))).until(finishedMotionMagic());
        return run(() -> leader.setControl(m_request.withPosition(6))).until(finishedMotionMagic());
    }
    public Command raiseLowAlgaePrep()
    {
        // return this.runOnce(
        //     () -> leader.setControl(m_request.withPosition(12))
        // );
    
        // return startEnd(() -> leader.setControl(m_request.withPosition(12)), () -> leader.setControl(m_request.withPosition(12))).until(finishedMotionMagic());
        return run(() -> leader.setControl(m_request.withPosition(2))).until(finishedMotionMagic());
    }
    

    public Command grabLowAlgae()
    {
        return this.runOnce(
            () -> leader.setControl(m_request.withPosition(4))
        );

        // if (DriveSubsystem.isCloseEnough().getAsBoolean())
        //     return run(() -> leader.setControl(m_request.withPosition(16))).until(finishedMotionMagic());//19.1
        // else
        //     return runOnce(() -> leader.setControl(m_request.withPosition(1.8)));
    }

    public Command grabHighAlage()
    {
        return this.runOnce(
            () -> leader.setControl(m_request.withPosition(8))
        );

        // if (DriveSubsystem.isCloseEnough().getAsBoolean())
        //     return run(() -> leader.setControl(m_request.withPosition(9))).until(finishedMotionMagic());//19.1
        // else
        //     return runOnce(() -> leader.setControl(m_request.withPosition(1.8)));
    }
    public Command trough(){
        return runOnce(()-> leader.setControl(m_request.withPosition(10)));
    }
    public Command DetermineAlgae(){
        return run(() -> leader.setControl(m_request.withPosition(algaeHeight))).until(finishedMotionMagic());
    }
    public Command DetermineAlgaePrep(){
        return run(() -> leader.setControl(m_request.withPosition(algaePrep))).until(finishedMotionMagic());
    }
    public Command Cancel(){
        return runOnce(() -> CommandScheduler.getInstance().cancelAll());
    }
    private double algaeHeight;
    private double algaePrep;
  private void updateHeightValue(){
   double TAGID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); 
   if (TAGID == 10 || TAGID == 21 || TAGID == 19 || TAGID == 17 || TAGID == 8 || TAGID == 6 )
    {
      algaeHeight = 4;
      algaePrep=2.5;
    }
    else if (TAGID == 20 || TAGID == 22 || TAGID == 18 || TAGID == 9 || TAGID == 11 || TAGID == 7)
    {
        algaeHeight = 8.25;
        algaePrep=5;
    }
    else {
        algaeHeight = 0;
        algaePrep=0;
    }
    
  }

    @Override
    public void periodic() {
        updateHeightValue();
        SmartDashboard.putNumber("elevator encoder", leader.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("target elevator encoder", m_request.Position);
        SmartDashboard.putBoolean("motionMagicB", finishedMotionMagic().getAsBoolean());
        SmartDashboard.putNumber("algae", algaeHeight);
        SmartDashboard.putNumber("algaeP", algaePrep);

        SmartDashboard.putBoolean("isCloseEnough", DriveSubsystem.isCloseEnough().getAsBoolean());

        // SmartDashboard.putBoolean("motion magic is running", ((BooleanSupplier) leader.getMotionMagicIsRunning()).getAsBoolean());

        // SmartDashboard.putNumber("setpoint",this.leader.getre);

        // Logger.getInstance().recordOutput("Elevator", leader.getPosition().getValueAsDouble());
        // System.out.println(((BooleanSupplier) leader.getMotionMagicIsRunning()).getAsBoolean());

    }
}
