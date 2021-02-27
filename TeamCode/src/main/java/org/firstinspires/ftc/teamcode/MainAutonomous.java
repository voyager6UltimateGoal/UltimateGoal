package org.firstinspires.ftc.teamcode;

import android.graphics.MaskFilter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Main Autonomous")
public class MainAutonomous extends LinearOpMode {
    Driving driver = new Driving(this);
    @Override
    public void runOpMode() {
        driver.initHwMap();
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        driver.resetEncoders();
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        driver.colorLightOn();
        Path[] initial = { // drive the robot into position to scan the rings
                new Path(M.STRAFE, D.BACKWARD, 0.15, 37),
        };
        driver.colorLightOff();
        driver.parseMoves(initial);
        Path[] noRing = { // what the robot does if there's no rings
                new Path(M.ROTATE, D.BACKWARD, 0.1, 90)
        };
        Path[] oneRing = { // what the robot does if there's one ring
//                new Path(M.ROTATE, D.FORWARD, 0.3, 90)
        };
        Path[] fourRing = { // what the robot does if there's four rings
//                new Path(M.ROTATE, D.FORWARD, 0.3, 90)
        };
//        int rings = driver.identifyRing();
        int rings = 0;
        if (rings == 0) {
            driver.parseMoves(noRing);
        } else if (rings == 1) {
            driver.parseMoves(oneRing);
        } else {
            driver.parseMoves(fourRing);
        }
//        sleep(15000);

    }
}