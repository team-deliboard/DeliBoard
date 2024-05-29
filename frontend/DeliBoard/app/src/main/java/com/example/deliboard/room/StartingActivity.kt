package com.example.deliboard.room

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.util.Log
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.constraintlayout.widget.ConstraintLayout
import com.example.deliboard.BaseApi
import com.example.deliboard.R
import com.example.deliboard.Tutorial
import com.example.deliboard.mainfg.MainActivity
import com.example.deliboard.login.LoginActivity
import com.example.deliboard.login.StartApi
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class StartingActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        if (isOccupied()) {
            val intent = Intent(this, MainActivity::class.java)
            startActivity(intent)
        } else {
            setContentView(R.layout.activity_starting)

            val startButton = findViewById<ConstraintLayout>(R.id.startButton)
            val guideButton = findViewById<ConstraintLayout>(R.id.guideButton)
            val logoutButton = findViewById<TextView>(R.id.logoutButton)

            logoutButton.setOnClickListener{
                showLogoutDialog()
            }

            guideButton.setOnClickListener{
                val intent = Intent(this, Tutorial::class.java)
                startActivity(intent)
            }

            startButton.setOnClickListener{
                roomStart()
            }
        }
    }

    private fun roomStart() {
        val client = BaseApi.getInstance().create(StartApi::class.java)

        val pref = applicationContext.getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        val storeId = pref.getString("storeId", null)
        val roomNum = pref.getString("roomNum", null)

        val requestBody = mapOf(
            "storeId" to storeId,
            "roomNumber" to roomNum,
        )

        client.sendRoomInfo(requestBody).enqueue(object : Callback<RoomStartResponse> {
            override fun onResponse(call : Call<RoomStartResponse>, response: Response<RoomStartResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null && responseBody.success) {
                        val intent = Intent(this@StartingActivity, MainActivity::class.java)
                        startActivity(intent)
                        Occupy(responseBody.roomLogId)
                        finish()
                        Log.d("Login", "login success")
                    } else {
                        Log.d("Login", "login failed $responseBody")
                        showFailure(this@StartingActivity, "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Login", "request failed $response")
                    showFailure(this@StartingActivity, "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<RoomStartResponse>, t: Throwable) {
                Log.e("Login", "$t")
                showFailure(this@StartingActivity, "요청에 실패했습니다.")
            }
        })
    }

    private fun showLogoutDialog() {
        val alertDialogBuilder = AlertDialog.Builder(this)
        alertDialogBuilder.apply {
            setTitle("연결해제")
            setMessage("현재 키오스크를 연결해제 합니다.")
            setPositiveButton("연결해제") {_, _ ->
                val sharedPreferences = getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)

                sharedPreferences.edit().remove("storeId").apply()
                sharedPreferences.edit().remove("roomNum").apply()

                val intent = Intent(this@StartingActivity, LoginActivity::class.java)
                intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP or Intent.FLAG_ACTIVITY_NEW_TASK)
                startActivity(intent)
                finish()

            }
            setNegativeButton("취소") {dialog, _ ->
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun isOccupied() : Boolean {
        val sharedPreferences = getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        return sharedPreferences.contains("isOccupied")
    }

    private fun showFailure(context: Context, message: String) {
        val builder = AlertDialog.Builder(context)

        builder.setTitle("인증 실패")
        builder.setMessage(message)

        builder.setPositiveButton("확인") {dialog, _ ->
            dialog.dismiss()
        }

        val dialog = builder.create()
        dialog.show()
    }

    private fun Occupy(roomLogId: Int) {
        val sharedPreferences = getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("isOccupied", "1")
        editor.putString("roomLogId", roomLogId.toString())
        editor.apply()
    }
}