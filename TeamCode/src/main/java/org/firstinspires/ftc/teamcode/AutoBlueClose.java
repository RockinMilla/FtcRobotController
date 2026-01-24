package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Close", group="Robot")
public class AutoBlueClose extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/23/2026, 5:38 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();
        r.launcherVelocity(820);

        shootBalls(r); // Shoot 1st set of balls

        // Pick up 2nd set of balls
        r.driveToPos(-100, -700, 0, 15, 3, 4);
        r.driveToPos(-730, -700, 0, 10, 2, 4);
        sleep(100);
        r.intakePower(0.8);
        r.turnLifterToDegrees(0, 600); // 800 was too fast
        r.driveToPos(-780, -100, 0, 10, 2, 2);
        r.waitForLifter();
        sleep(100);

        shootBalls(r); // Shoot 2nd set of balls

        // Pick up 3rd set of balls
        r.driveToPos(-100, -800, 0, 10, 2, 4);
        r.driveToPos(-1310, -800, 0, 10,2,4);
        r.intakePower(0.8);
        //sleep(100);
        r.turnLifterToDegrees(0, 550);
        r.driveToPos(-1390, 150, 0, 10, 2, 1.5); // Pick up
        r.waitForLifter();
        sleep(100);

        r.driveToPos(-1200, -300, 0); // move to avoid gate
        r.driveToPos(-1200, -100, 0); // empty gate
        shootBalls(r); // Shoot 3rd set of balls

        // Pick up 4th set of balls
        r.driveToPos(-100, -800, 0, 10, 2, 3);
        r.driveToPos(-1870,-800,0,10,2,3);
        r.intakePower(0.8);
        //sleep(100);
        r.turnLifterToDegrees(0,600);
        r.driveToPos(-1920,150,0,10,2,1.5);
        r.waitForLifter();
        sleep(100);

        r.driveToPos(-1350, -600, 0); // move to avoid gate
        shootBalls(r); // Shoot 4th set of balls

        r.driveToPos(0, -1000, 0, 20, 5, 2); // Park

        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(-100, -700, -33, 20, 2, 5); // Go to shooting position
        r.driveToPos(-100, -700, -33, 15, 1, 1); // Intentionally repeated
        r.intakePower(0.9); // Propell balls
        r.turnLifterToDegrees(360, 1400); // Shoot
        r.turnLifterByDegrees(-10, 1400);
        r.turnLifterToDegrees(360, 1400);
        r.waitForLifter();
        sleep(100); // Necessary to ensure that all balls are shot
        r.intakePower(0);
        r.turnLifterToDegrees(-190); // Reset lifter
    }
}