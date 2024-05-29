package com.example.deliboard.mainfg

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.navigation.findNavController
import com.example.deliboard.MainNavDirections
import com.example.deliboard.R

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

//        hideSystemUI()
        setContentView(R.layout.activity_main)


        val logoButton = findViewById<ConstraintLayout>(R.id.logoButton)
        logoButton.setOnClickListener{
            val action = MainNavDirections.actionGlobalHome()
            findNavController(R.id.mainFragmentView).navigate(action)
        }

    }

    private fun showBackButton() {

    }
}