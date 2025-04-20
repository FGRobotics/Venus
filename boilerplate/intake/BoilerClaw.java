package org.firstinspires.ftc.teamcode.boilerplate.intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BoilerClaw {
    public ServoImplEx base,joint,wrist,claw;
    public BoilerServoEncoder jointPos, basePos;
    public BoilerClaw(HardwareMap hardwareMap){
        base = hardwareMap.get(ServoImplEx.class, "base");
        joint = hardwareMap.get(ServoImplEx.class, "joint");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        jointPos = new BoilerServoEncoder(hardwareMap, "jointpos");
        basePos = new BoilerServoEncoder(hardwareMap, "basepos");
    }
    public void setWristMiddle(){
        wrist.setPosition(0.62);
    }
    public void setINTAKE_SCAN(){
        base.setPosition(0.265);
        joint.setPosition(0.93);
        setWristMiddle();
    }
    public void noWristScan(){
        base.setPosition(0.265);
        joint.setPosition(0.93);
        openClaw();
    }
    public void setINTAKE_TRANSFER(){
        setWristMiddle();
        base.setPosition(0.33);
        joint.setPosition(0.05);
    }
    public void setINTAKE_PICKUP(){
        openClaw();
        joint.setPosition(0.87);
        base.setPosition(0.145);
    }
    public void setINTAKE_Middle(){
        setWristMiddle();
        joint.setPosition(0.43);
        base.setPosition(0.08);
        openClaw();
    }
    public void setINTAKE_WALL(){
        setWristMiddle();
        joint.setPosition(0.775);
        base.setPosition(0.7);
        openClaw();
    }
    public void setINTAKE_WALL_UP(){
        closeClaw();
        sleep(200);
        setWristMiddle();
        joint.setPosition(0.4);
        base.setPosition(0.7);
    }
    public void setSUBMERSIBLEZONERETRACT(){
        setWristMiddle();
        joint.setPosition(0.65);
        base.setPosition(0.4);
    }
    public void noWristERETRACT(){
        closeClaw();
        sleep(200);
        joint.setPosition(0.65);
        base.setPosition(0.4);
    }
    public void setINTAKE_START(){
        setWristMiddle();
        joint.setPosition(0.8);
        base.setPosition(0.7);
        openClaw();
    }
    public void specimenFloor(){
        openClaw();
        base.setPosition(0.13);
        joint.setPosition(0.8);
        sleep(200);
    }
    public void specimenFloorUp(){
        openClaw();
        base.setPosition(0.21);
        joint.setPosition(0.8);
    }
    public void sleep(long mills){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() < mills){}
    }
    public void closeClaw(){
        claw.setPosition(0);
    }
    public void openClaw(){
        claw.setPosition(0.9);
    }

    public void pickup(){
        sleep(300);
        setINTAKE_PICKUP();
        sleep(300);
        closeClaw();
        sleep(300);
        setSUBMERSIBLEZONERETRACT();
    }
    public void speciGroundPickup(){
        specimenFloorUp();
        sleep(1200);
        specimenFloor();
        sleep(200);
        closeClaw();
        sleep(200);
    }
}
