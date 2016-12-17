package com.example.android.registeruser;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.ImageButton;

/**
 * Created by Vivek on 2016-07-14.
 */
public class landing_page extends AppCompatActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // Set View to register.xml
        setContentView(R.layout.landing_page);


        ImageButton controlEllo = (ImageButton) findViewById(R.id.ellocontrolicon);
        ImageButton elloanalytics = (ImageButton) findViewById(R.id.elloanalystics);
        ImageButton ellomaintainace = (ImageButton) findViewById(R.id.ellomaintainance);


       //Listening to each button and then opening up respective page as clicked

        controlEllo.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to register screen now
                Intent goToControl = new Intent(getApplicationContext(), bluetoothConnector.class);
                startActivity(goToControl);
            }
        });

       elloanalytics.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to register screen now
                Intent goToAnalytics = new Intent(getApplicationContext(), getanalytics.class);
                startActivity(goToAnalytics);
            }
        });

        ellomaintainace.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to register screen now
                Intent goToMaintainance = new Intent(getApplicationContext(), maintainance.class);
                startActivity(goToMaintainance);
            }
        });




    }
}





