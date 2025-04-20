package org.firstinspires.ftc.teamcode.boilerplate.tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
public class ExperimentalTester extends LinearOpMode {
    Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(2); // Switch to pipeline number 2
        limelight.start(); // This tells Limelight to start looking
        waitForStart();
        while(opModeIsActive()) {
            if(getRuntime() > 3) {
                try {
                    telemetry.addData("running: ", limelight.isRunning());
                    telemetry.addLine(limelight.getStatus().toString());
                    LLResult results = limelight.getLatestResult();
                    telemetry.addData("pl: ", results.getPipelineIndex());
                    telemetry.update();

                } catch (Exception e) {
                    telemetry.addData("exception: ", e.toString());
                    telemetry.update();
                }
            }
        }
    }
}
