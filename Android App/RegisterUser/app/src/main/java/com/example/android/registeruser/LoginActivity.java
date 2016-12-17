package com.example.android.registeruser;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.TextView;

public class LoginActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //setting default screen to login.xml
        setContentView(R.layout.login);


        //defining the xml file we want this activity to go into
        TextView registerScreen = (TextView) findViewById(R.id.link_to_register);
        TextView landing_screen = (TextView) findViewById(R.id.btnLogin);


        //Listening to register new user account link
       registerScreen.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to register screen now
                Intent i = new Intent(getApplicationContext(), RegisterActivity.class);
                startActivity(i);
            }
        });


        //Listening to login to send to new landing page
      landing_screen.setOnClickListener(new View.OnClickListener() {

            public void onClick(View v) {
                //switching to landing page screen now
                Intent navigateToLandingPage = new Intent(getApplicationContext(), landing_page.class);
                startActivity(navigateToLandingPage);
            }
        });
    }
}




