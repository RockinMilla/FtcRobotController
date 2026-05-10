package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Far", group="Robot")
public class AutoRedFar extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/26/2026, 12:53 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();

        r.intakePower(0.5);
        r.launcherVelocity(1200); // Change this when changing launcher velocity

        shootBalls(r); // Shoot preloaded balls

        r.driveToPos(100, 750, -95, 10, 2, 4);
        r.driveToPos(1250, 750, -95, 10,2,4, true);
        r.intakePower(0.8);
        //sleep(100);
        //r.driveToPos( -1390, 150, 90, 10, 2, 1.5); // Pick up
        r.waitForLifter();
        sleep(100);

        shootBalls(r); // Shoot 2nd set of balls

        r.driveToPos(1200, 300, -170, 10, 2, 4);
        r.driveToPos(1250, 20, -170, 10, 2, 4, true);

        shootBalls(r);

        r.driveToPos(500, 200, -70); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r) {
        r.driveToPos(-50, 150, -18, 10, 1.5 ,5);
        r.waitForLaunchers(1200); // Change this when changing launcher velocity
        sleep(500);
        r.intakePower(0.5);
        r.turnLifterToDegrees(360);
        r.waitForLifter();
        sleep(400);
        r.turnLifterToDegrees(-150); // Reset lifter
    }
}