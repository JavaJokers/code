package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

private static final String VUFORIA_KEY = "ATuJKTT/////AAABmfYsY1Jl3UGAsY0tNaK6VX1n3rDiPMxjjypA43x2n2q9ogQYwfcCmJGOOeUI3QzFwEp7uplOIdMXu4XNVztl2Y/fNeykMAkE5GkkjfLb0IBVQ1oQBFVlLEcyjK024nd9xqEJhMmffZhk4RRViJHrHAKYaISPfiogrjqmEWsad5pMt9pczFQbbtqoZLoDfQ3kDvKwPx4VWAe8E40eZqMF/+XbTCb4xj/OyPEZjxPiEnAxSLEtgAk4O6G6PjWvy+/m0Kav4+vWeUwnNi42RBvclYnJebeJlsFDoczVFgkeNVp3u1Vf3QBaFTsay3GwyCGRREZ+l9K0HMbRpNxfEkUzruBOxIfyle8Rr6EiPFY6MX4w ";

@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")

public class MyTensorFlowExample extends LinearOpMode {
    initVuforia();
    initTfod();
    private void initVuforia() {

    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection = CameraDirection.BACK;

    vuforia = ClassFactory.getInstance().createVuforia(parameters);

   
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    if (tfod != null) {
        tfod.activate();
        tfod.setZoom(2.5, 16.0/9.0);
    }
}