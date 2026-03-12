
package frc.robot.subsystems;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake {
    private TalonFX wrist;
    private TalonFX wheel;
    private static final double startPosition = -2; //change later
    private static final double endPosition = -10; ///change later
    //private final CANBus canbus;
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    
    public Intake(int CanID1, int CanID2){
        //canbus = new CANBus("rio");
        wheel = new TalonFX(CanID1);
        wrist = new TalonFX(CanID2);

        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = 0.3; //.3
        slot0Configs.kS = 0; //.00
        slot0Configs.kV = 0.01; // 0.01
        slot0Configs.kA = 0.01; //0.1
        slot0Configs.kP = 2.8; //2.8
        slot0Configs.kI = 0; //00
        slot0Configs.kD = 0.1;


        // Set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.12; //.12
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
}

