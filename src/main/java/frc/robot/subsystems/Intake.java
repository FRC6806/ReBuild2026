
package frc.robot.subsystems;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake {
    private TalonFX wrist;
    private TalonFX wheel;
    private static final double startPosition = 0; //change later
    private static final double endPosition = -4.2; ///change later
    //private final CANBus canbus;
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    
    public Intake(int CanID1, int CanID2){
        //canbus = new CANBus("rio");
        wheel = new TalonFX(CanID1);
        wrist = new TalonFX(CanID2);
        wrist.setPosition(0);

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true));
        var slot0Configs = talonFXConfigs.Slot0;
        talonFXConfigs.MotorOutput.DutyCycleNeutralDeadband = .03;
        slot0Configs.kG = 0.45; //.3
        slot0Configs.kS = 0; //.00
        slot0Configs.kV = 0.0; // 0.01
        slot0Configs.kA = 0.; //0.1
        slot0Configs.kP = 1; //2.8
        slot0Configs.kI = 0; //00
        slot0Configs.kD = 0.1;
        slot0Configs.withGravityType(GravityTypeValue.Arm_Cosine);


        // Set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.05; //.12
        motionMagicConfigs.MotionMagicExpo_kA = 0.1; //.1


        // Apply configurations to the motor
        wrist.getConfigurator().apply(talonFXConfigs);
    }


    public void setWheelSpeed(double speed){
        wheel.set(speed);
    }

    public void wristExtend(){
        wrist.setControl(m_request.withPosition(endPosition));
    }
    public void wristRetract(){
        wrist.setControl(m_request.withPosition(startPosition));
    }

    public void wristShake() {
        double currentTime = Timer.getFPGATimestamp();
        if((currentTime % 2) > 1 ) {
            wrist.setControl(m_request.withPosition(-2));
        } else {
            wristExtend();
        }

    }

    public double getIntakePosition() {
        return wrist.getPosition().getValueAsDouble();
    }

}

