package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;

@Autonomous(name = "auto2", group= "something")
public class auto2 extends LinearOpMode {
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backRight = null;
    DcMotor backLeft = null;

    @Override
    public void runOpMode()  {
        frontLeft = hardwareMap.get(DcMotor.class, "motor0");
        frontRight = hardwareMap.get(DcMotor.class, "motor2");
        backLeft = hardwareMap.get(DcMotor.class, "motor1");
        backRight = hardwareMap.get(DcMotor.class, "motor3");

        waitForStart();

        while (opModeIsActive()) {
            moveForward(0.7, 500);
            stop();
            turnRight(0.7, 1000);
            stop();
            moveForward(0.7, 2000);
            stop();
        }

    }

    public void moveForward(double power, long time) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(time);

    }

    public void turnRight(double power, long time) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(time);

    }
}
