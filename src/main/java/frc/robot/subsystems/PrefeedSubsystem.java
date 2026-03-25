package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PrefeedSubsystem extends SubsystemBase {

    private final SparkMax leader;
    private final SparkMax follower1;
    private final SparkMax follower2;

    @SuppressWarnings("removal")
    public PrefeedSubsystem() {
        leader = new SparkMax(12, MotorType.kBrushless);
        follower1 = new SparkMax(13, MotorType.kBrushless);
        follower2 = new SparkMax(14, MotorType.kBrushless);

        SparkMaxConfig baseConfig = new SparkMaxConfig();
        baseConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.apply(baseConfig);
        leaderConfig.inverted(false);

        SparkMaxConfig follower1Config = new SparkMaxConfig();
        follower1Config
            .apply(baseConfig)
            .follow(leader, true);

        SparkMaxConfig follower2Config = new SparkMaxConfig();
        follower2Config
            .apply(baseConfig)
            .follow(leader, false);

        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower1.configure(follower1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower2.configure(follower2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        leader.set(speed);
    }

    public void stop() {
        leader.set(0);
    }

    public Command runPrefeed(double speed) {
    return Commands.startEnd(
        () -> setSpeed(speed),
        () -> stop(),
        this
    );
}
}