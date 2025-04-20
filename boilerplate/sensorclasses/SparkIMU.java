package org.firstinspires.ftc.teamcode.boilerplate.sensorclasses;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SparkIMU {
    public SparkFunOTOS otos;
    public SparkIMU(HardwareMap hardwareMap){
        otos = hardwareMap.get(SparkFunOTOS.class, "SparkFunIMU");
        intializeImu();
        resetIMU();
    }
    public void intializeImu(){
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(1.375, -3.625, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.002823409);
        otos.calibrateImu();
        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otos.resetTracking();
    }
    public void resetIMU(){
        otos.resetTracking();
    }
}
