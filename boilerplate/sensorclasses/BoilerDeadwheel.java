package org.firstinspires.ftc.teamcode.boilerplate.sensorclasses;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BoilerDeadwheel {
    public DcMotorEx deadwheel;
    public BoilerDeadwheel(DcMotorEx motor){
        deadwheel = motor;
    }

    public int getCurrentPosition(){return deadwheel.getCurrentPosition();}
    public void reset(){
        deadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean isStopped(){
        return Math.abs(deadwheel.getVelocity()) < 100;
    }
}

//one that resets to 0 and runs without encoders
