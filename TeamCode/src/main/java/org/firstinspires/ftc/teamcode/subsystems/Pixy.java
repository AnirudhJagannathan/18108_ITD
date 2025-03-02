package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class Pixy {
    //Declare a new I2cDeviceSync
    private I2cDeviceSynch pixyCam;
    private byte[] sign1;
    private byte[] sign2;
    private LinearOpMode opMode;
    //Get name from hardware config
    public Pixy(LinearOpMode opMode, HardwareMap hardwareMap){
        pixyCam = hardwareMap.i2cDeviceSynch.get("pixyCam");
        this.opMode = opMode;
    }

    // Now to get data
    // Signature Query
    // This query simply asks Pixy to return the largest detected block of the specified signature. The
    // specified signature is the Query Address ­ 0x50. For example, a Query Address of 0x51 would
    //request the largest detected block of signature 1.
    //Query Address Optional Query Data Pixy Response
    //0x51 thru 0x57 none 5 bytes that describe the largest detected block within all signatures 1 thru 7. If no object is detected, all bytes will be 0.
    //Byte Description
    //0 Number of blocks that match the specified signature.
    //1 X value of center of largest detected block, ranging between 0 and 255. An x value of 255 is the far right­side of Pixy’s image.
    //2 Y value of center of largest detected block, ranging between 0 and 199. A value of 199 is the far bottom­side of Pixy’s image.
    //3 Width of largest block, ranging between 1 and 255. A width of 255 is the full width of Pixy’s image.
    //4 Height of largest block, ranging between 1 and 200. A height of 200 is the full height of Pixy’s image.
    //All of this info and more can be found at cmucam.org/attachments/1290/Pixy_LEGO_Protocol_1.0.pdf
    public void color_Detection() {
        pixyCam.engage();
        sign1 = pixyCam.read(0x51, 5);
        sign2 = pixyCam.read(0x52, 5);
        //notice the 0xff&sign1[x], the 0xff& does an absolute value on the byte
        //the sign1[x] gets byte 1 from the query, see above comments for more info
        opMode.telemetry.addData("X value of sign1", 0xff & sign1[1]);
        opMode.telemetry.addData("X value of sign2", 0xff & sign2[1]);
    }
}
