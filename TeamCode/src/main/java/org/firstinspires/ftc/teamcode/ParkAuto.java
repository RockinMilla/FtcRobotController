package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Park Auto", group="Robot")
public class ParkAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Forward and right 20 cm diagonally");
        telemetry.addData("This code was last updated", "11/10/2026, 1:39 pm");
        telemetry.update();
        waitForStart();

        robot.driveToPos(200, 150, 0, 50, 5, 3);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}