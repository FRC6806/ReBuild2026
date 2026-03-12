
package frc.robot.subsystems;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


public class Climb {
    private TalonFX climb1;
    private TalonFX climb2;
    private final CANBus canbus;
    private static final int climbPosition = 10;
    private static final int startPosition = 0;
   
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);


    public Climb(int CanID1, int CanID2){
        canbus = new CANBus("idk");
        TalonFX c1 = new TalonFX(CanID1, canbus);
        TalonFX c2 = new TalonFX(CanID2, canbus);
        climb1 = c1;
        climb2 = c2;


        var talonFXConfigs = new TalonFXConfiguration();


        // Set slot 0 gain
        //CHANGE THESE VALUES!!!!! We stole code from last yea4!
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = 0.5; //.3
        slot0Configs.kS = 0.6; //.00
        slot0Configs.kV = 0.3; // 0.01
        slot0Configs.kA = 0.3; //0.1
        slot0Configs.kP = 0.75; //4.8
        slot0Configs.kI = 0.2; //00
        slot0Configs.kD = 0.1;


        // Set Motion Magic Expo settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.5; //.12
        motionMagicConfigs.MotionMagicExpo_kA = 0.6; //.1


        // Apply configurations to the motor
        climb1.getConfigurator().apply(talonFXConfigs);


    }
    public void climbSpeed(double speed){
         climb1.set(speed);
         climb2.set(speed);
    }


    public void setClimbUp(int climbPosition){
        climb1.setControl(m_request.withPosition(climbPosition));
        climb2.setControl(m_request.withPosition(climbPosition));
    }
    public void setClimbDown(int climbPosition){
        climb1.setControl(m_request.withPosition(startPosition));
        climb2.setControl(m_request.withPosition(startPosition));
    }
}


