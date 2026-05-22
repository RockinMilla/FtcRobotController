package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Far", group="Robot")
public class AutoRedFar extends LinearOpMode {
    private double launcherVelocity = 1220; // Change this when changing launcher velocity
    private double defaultIntakePower = 1; // Change this when changing pickup intake power

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "5/19/2026, 11:43 am"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.intakePower(defaultIntakePower);
        r.launcherVelocity(launcherVelocity);
        r.setpValueLifter(20);

        // 1st set of balls
        shootBalls(r);

        // 2nd set of balls
        r.driveToPos(0, 700, -95, 10, 2, 2);
        r.driveToPos(1150, 700, -95, 10, 2, 2, true);
        sleep(500);
        shootBalls(r);

        // 3rd set of balls
        r.driveToPos(1175,  150, -110, 10, 2, 2, true);
        sleep(200);
        shootBalls(r);

        // 4th set of balls
        r.driveToPos(1175,  150, -110, 10, 2, 2, true);
        sleep(200);
        shootBalls(r);

        // park
        r.driveToPos(500, 200, -70);
        r.launcherVelocity(0);
        r.intakePower(0);
        r.setpValueLifter(10);
    }

    private void shootBalls(RockinBot r) {
        r.driveToPos(-50, 150, -15, 10, 1.5, 3);
        r.waitForLaunchers(launcherVelocity); // Change this when changing launcher velocity
        sleep(500);
        r.turnLifterToDegrees(360, 2000);
        r.waitForLifter();
        sleep(100);
        r.turnLifterByDegrees(-10);
        r.turnLifterToDegrees(360, 2000);
        sleep(200);
        r.turnLifterToDegrees(-150, 2000); // Reset lifter
    }
}