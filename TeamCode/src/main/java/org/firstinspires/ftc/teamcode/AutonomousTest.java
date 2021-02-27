package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name="Color Sensor Test")
public class AutonomousTest extends LinearOpMode {
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
        telemetry.addData("Status", "Moving to Detection");
        telemetry.update();
        Path[] paths = {
                new Path(M.STRAFE, D.BACKWARD, 0.11, 38),
        };
        driver.parseMoves(paths);
        int rings = driver.identifyRing();
        telemetry.addData("Rings", rings);
        telemetry.update();
        sleep(15000);
    }
}