package com.example.deliboard

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.ImageView
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import com.example.deliboard.mainfg.MainActivity

class Tutorial : AppCompatActivity() {

    private lateinit var imageView: ImageView
    private lateinit var imageView2: ImageView

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_tutorial)

        imageView = findViewById(R.id.tutorialImage)
        imageView2 = findViewById(R.id.tutorialImage2)

        imageView2.visibility = View.GONE

        imageView.setOnClickListener{
            imageView.visibility = View.GONE
            imageView2.visibility = View.VISIBLE
        }

        imageView2.setOnClickListener{
//            val intent = Intent(this, MainActivity::class.java)
//            startActivity(intent)
            finish()
        }
    }
}