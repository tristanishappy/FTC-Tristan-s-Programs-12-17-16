package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

public class AutonomousSimpleForwardJustShoot extends LinearOpMode
{
    //Declares name of DcMotors
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick

    //LightSensor light;

    DcMotor motorPop;
    DcMotor motorCol;

    //DcMotor motorUL;
    //DcMotor motorUR;
    private double lift = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //light = hardwareMap.lightSensor.get("light");

        motorRF = hardwareMap.dcMotor.get("motorRF");//sets DcMotors to type of motor
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorPop = hardwareMap.dcMotor.get("popper");
        motorCol = hardwareMap.dcMotor.get("collector");

        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);

        //buttonColor.enableLed(true);
        //Movement
        mecaMovement(270,0.5);
        sleep(800);
        mecaMovement(0,0);

        motorPop.setPower(1);
        sleep(1500);
        motorPop.setPower(0);
        sleep(100);
        motorCol.setPower(1);
        sleep(3000);
        motorCol.setPower(0);
        sleep(1000);
        motorPop.setPower(1);
        sleep(1500);
        motorPop.setPower(0);
        sleep(500);
        mecaMovement(0,0);
        sleep(100);

        mecaMovement(90,0.5);
        sleep(850);
        mecaMovement(0,0);
        sleep(500);


    }

    public void mecaMovement(double degrees, double power)
    {
        double radians = Math.toRadians(degrees+45);
        motorLF.setPower(power * Math.cos(radians));
        motorLB.setPower(power * Math.sin(radians));
        motorRF.setPower(power * Math.sin(radians));
        motorRB.setPower(power * Math.cos(radians));
    }
    public void movement(double powerL, double powerR)
    {
        motorLF.setPower(powerL);
        motorLB.setPower(powerL);
        motorRF.setPower(powerR);
        motorRB.setPower(powerR);
    }
}