package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "start")
public class LessonCode extends LinearOpMode {
    public DcMotor motor;
    public Servo servo;
    public CRServo CRservo;


public void runOpMode(){
    motor = hardwareMap.get(DcMotor.class, "motor");
    servo = hardwareMap.get(Servo.class, "servo");
    CRservo = hardwareMap.get(CRServo.class,"CRservo");

    waitForStart();

    while (opModeIsActive()){

        motor.setPower(gamepad1.left_stick_y);

        //servo range is from 0e-1
        if (gamepad1.b){
            servo.setPosition(0.5);
        }

        if (gamepad1.y){
            CRservo.setPower(0.4);
            } else {
            CRservo.setPower(0);
        }

        telemetry.addData("motor power", motor.getPower());
        telemetry.update();

        }
    }
}
