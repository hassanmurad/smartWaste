package com.example.android.registeruser;

import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.ImageView;

/**
 * Created by Vivek on 2016-07-15.
 */
public class maintainance extends AppCompatActivity {


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //setting default screen to login.xml
        setContentView(R.layout.maintainance);

        ImageView callMaintainance = (ImageView) findViewById(R.id.phonebtn);
        ImageView infoOnTouchlessSensor = (ImageView) findViewById(R.id.touchlessSensorbtn);
        ImageView infoOnResidualSpaceSensor = (ImageView) findViewById(R.id.residualSpaceSensorbtn);
        ImageView infoOnLidMotor = (ImageView) findViewById(R.id.lidMotorbtn);
        ImageView infoOnMotorizedTray = (ImageView) findViewById(R.id.motorizedTrayButton);

        callMaintainance.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                Intent intent = new Intent(Intent.ACTION_DIAL);
                intent.setData(Uri.parse("tel:6043654180"));
                if (intent.resolveActivity(getPackageManager()) != null) {
                    startActivity(intent);
                }
            }


        });

        infoOnTouchlessSensor.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to register screen now
                Intent goToTouchLessSensorInfo = new Intent(getApplicationContext(), touchless_sensor_info.class);
                startActivity(goToTouchLessSensorInfo);
            }
        });

        infoOnResidualSpaceSensor.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to register screen now
                Intent goToResidualSpaceSensorInfo = new Intent(getApplicationContext(), residual_space_sensor.class);
                startActivity(goToResidualSpaceSensorInfo);
            }
        });

       infoOnLidMotor.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to register screen now
                Intent goToLidMotorInfo = new Intent(getApplicationContext(), lid_motor_info.class);
                startActivity(goToLidMotorInfo);
            }
        });

       infoOnMotorizedTray.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to register screen now
                Intent goToLidMotorInfo = new Intent(getApplicationContext(), motorized_tray_info.class);
                startActivity(goToLidMotorInfo);
            }
        });

    }
}
