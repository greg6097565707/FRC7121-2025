package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climber extends SubsystemBase {
     private SparkMax m_spark;
     public climber()
    {
        m_spark = new SparkMax(Constants.ClimbConstants.MotorID , MotorType.kBrushless);
        //irSensor = new DigitalInput(0);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

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
        m_spark.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    public Command Forward(){
        return run(()-> m_spark.set(0.5));
    }
    public Command Back(){
        return run(()-> m_spark.set(-0.5));
    }
    public Command stopClimber(){
        return run(()-> m_spark.set(0));
    }
}
