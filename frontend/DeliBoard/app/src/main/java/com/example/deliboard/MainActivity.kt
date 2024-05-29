package com.example.deliboard

import FCM.MessagingService
import android.content.Context
import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.TextView

class MainActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        if (!isLogined()) {
            val intent = Intent(this, LoginActivity::class.java)
            startActivity(intent)
        } else {
            setContentView(R.layout.activity_main)
        }


        // 켤때 FCM 토큰 가져오는거
        MessagingService().getFirebaseToken()

        val rightTextView = findViewById<TextView>(R.id.rightTextView)
//        val backButton = findViewById<ImageView>(R.id.backButton)

        //내비 감지 -> 메인프래그먼트가 아니면 뒤로가기 버튼 보이게
//        val navController = findNavController(R.id.fragmentContainerView)
//        navController.addOnDestinationChangedListener{_, destnation, _ ->
//            val currentFragmentId = destnation.id
//
//            if (currentFragmentId == androidx.constraintlayout.widget.R.id.home) {
//                backButton.visibility = View.GONE
//                rightTextView.visibility = View.VISIBLE
//            } else {
//                rightTextView.visibility = View.GONE
//                backButton.visibility = View.VISIBLE
//                showBackButton()
//            }
//        }
    }

    private fun isLogined() : Boolean {
        val sharedPreferences = getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        return sharedPreferences.contains("storeName") && sharedPreferences.contains("storeTable")
    }

    private fun showBackButton() {

    }
}