package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax leader;
    private SparkMax follower;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private int targetPosition;

    public ElevatorSubsystem() {
        leader = new SparkMax(Constants.ElevatorConstants.leaderID , MotorType.kBrushless);
        follower = new SparkMax(Constants.ElevatorConstants.followerID, MotorType.kBrushless);

        targetPosition = 30;

        closedLoopController = leader.getClosedLoopController();
        encoder = leader.getEncoder();

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        // leaderConfig.encoder.setPositionConversionFactor(1);
        leaderConfig.encoder.positionConversionFactor(1);

        /*
        * Configure the closed loop controller. We want to make sure we set the
        * feedback sensor as the primary encoder.
        */
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.013)
            .i(0.)
            .d(0)
            .outputRange(-1, 1);
            // Set PID values for velocity control in slot 1
            // .p(0, ClosedLoopSlot.kSlot1)
            // .i(0, ClosedLoopSlot.kSlot1)
            // .d(0, ClosedLoopSlot.kSlot1)
            // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
            // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        leaderConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.
            .maxVelocity(10000)
            .maxAcceleration(20000)
            .allowedClosedLoopError(0.5);
            // Set MAXMotion parameters for velocity control in slot 1
            // .maxAcceleration(500, ClosedLoopSlot.kSlot1)
            // .maxVelocity(6000, ClosedLoopSlot.kSlot1)
            // .allowedClosedLoopError(3, ClosedLoopSlot.kSlot1);


        /*
        * Set parameters that will apply to all SPARKs. We will also use this as
        * the left leader config.
        */
        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        // Apply the global config and invert since it is on the opposite side
        leaderConfig
            .apply(globalConfig);

        // Apply the global config and set the leader SPARK for follower mode
        followerConfig
            .apply(globalConfig)
            .follow(leader, true);

        /*
        * Apply the configuration to the SPARKs.
        *
        * kResetSafeParameters is used to get the SPARK MAX to a known state. This
        * is useful in case the SPARK MAX is replaced.
        *
        * kPersistParameters is used to ensure the configuration is not lost when
        * the SPARK MAX loses power. This is useful for power cycles that may occur
        * mid-operation.
        */
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command raiseElevatorTop()
    {
        return this.runOnce(
            () -> 
            closedLoopController.setReference(40, ControlType.kMAXMotionPositionControl,//50
          ClosedLoopSlot.kSlot0));
    }

    public Command raiseElevatorMid()
    {
        return this.runOnce(
            () -> 
            closedLoopController.setReference(15, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0));
    }

    public Command lowerElevator()
    {
        return this.runOnce(
            () -> 
            closedLoopController.setReference(0, ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0));
    }

    public int getTargetPosition()
    {
        return targetPosition;
    }


     @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator encoder", leader.getEncoder().getPosition());
        
    }
}
