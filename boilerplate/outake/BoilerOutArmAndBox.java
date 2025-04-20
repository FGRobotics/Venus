package org.firstinspires.ftc.teamcode.boilerplate.outake;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.boilerplate.helpers.Control;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BoilerOutArmAndBox {
    ServoImplEx outtakeArm, outtakeBox;
    public DistanceSensor color;
    public boolean colorSensorStaticy = false;

    public BoilerOutArmAndBox(HardwareMap hardwareMap){
        outtakeArm = hardwareMap.get(ServoImplEx.class, "outtakeArm");
        outtakeBox = hardwareMap.get(ServoImplEx.class, "outtakeBox");
        color = hardwareMap.get(DistanceSensor.class, "color");
    }
    public void setOUTTAKE_ARM_DOWN(){
        outtakeArm.setPosition(0);
    }
    public void setOUTTAKE_ARM_UP(){
        outtakeArm.setPosition(1);
    }
    public void setOUTTAKE_ARM_TRANSFER(){
        outtakeArm.setPosition(0.2717);
    }
    public void setOUTTAKE_TRANSFER(){
        outtakeBox.setPosition(0.37);
    }
    public void setOUTTAKE_TRANSFER_ADJUSTED(){
        outtakeBox.setPosition(0.44);
    }

    public void setOUTTAKE_SPECIMEN(){
        outtakeBox.setPosition(0.63);
    }
    public void setOUTTAKE_DROP(){
        outtakeBox.setPosition(0);
    }
    public void setOUTTAKE_AUTO_DROP(){
        outtakeBox.setPosition(0.18);
    }


    public void setOUTTAKE_MECH_TRANSFER(){
        setOUTTAKE_TRANSFER_ADJUSTED();
        setOUTTAKE_ARM_TRANSFER();
    }
    public void setOUTTAKEFULLSPECI(){
        setOUTTAKE_ARM_UP();
        setOUTTAKE_SPECIMEN();
    }
    public void autoBasketDropoff(){
        setOUTTAKE_AUTO_DROP();
        setOUTTAKE_ARM_UP();
        Control.sleep(1);
        setOUTTAKE_MECH_TRANSFER();
        Control.sleep(0.3);
    }
    public void simmedSpeciDropoff(){
        setOUTTAKEFULLSPECI();
        setOUTTAKE_DROP();
        Control.sleep(1);
        setOUTTAKE_MECH_TRANSFER();
        Control.sleep(0.3);
    }
    public boolean hasObject(){
        if(!colorSensorStaticy){
            double val = color.getDistance(DistanceUnit.CM);
            if(val > 5000){
                colorSensorStaticy = true;
            }
            return val < 5.5;
        }
        return false;

    }
    /*public List<Integer> returnRGB(){
        List<Integer> vals = new ArrayList<>();
        vals.add(color.blue());
        vals.add(color.red());
        vals.add(color.green());
        return vals;
    }
     */

}
