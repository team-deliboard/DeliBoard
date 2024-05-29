package com.example.deliboard.login

import com.example.deliboard.firebase.MessagingService
import android.content.Context
import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.constraintlayout.widget.ConstraintLayout
import com.example.deliboard.BaseApi
import com.example.deliboard.R
import com.example.deliboard.room.StartingActivity
import com.example.deliboard.SuccessResponse
import com.example.deliboard.manager.Manager
import com.google.android.material.textfield.TextInputEditText
import retrofit2.Call
import retrofit2.Response
import retrofit2.Callback

class LoginActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        if (isLogined()) {
            val intent = Intent(this, StartingActivity::class.java)
            startActivity(intent)
        } else if (isManager()){
            val intent = Intent(this, Manager::class.java)
            startActivity(intent)
        } else {
            setContentView(R.layout.activity_login)
        }

        // 켤때 FCM 토큰 가져오는거
        MessagingService().getFirebaseToken()

        val loginButton = findViewById<ConstraintLayout>(R.id.loginButton)
        val storeNumInput = findViewById<TextInputEditText>(R.id.storeNumField)
        val roomNumInput = findViewById<TextInputEditText>(R.id.roomNumField)
        val managerButton = findViewById<TextView>(R.id.managerButton)

        loginButton.setOnClickListener {
            val storeId = storeNumInput.text.toString()
            val roomNum = roomNumInput.text.toString()

            StoreNumCheck(storeId, roomNum)
        }

        managerButton.setOnClickListener {
            val storeId = storeNumInput.text.toString()

            ManagerLogin(storeId)
        }
    }

    private fun isLogined() : Boolean {
        val sharedPreferences = getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        return sharedPreferences.contains("storeId") && sharedPreferences.contains("roomNum")
    }

    private fun isManager() : Boolean {
        val sharedPreferences = getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        return sharedPreferences.contains("isManager")
    }

    private fun StoreNumCheck(storeId: String, roomNum: String) {
        val client = BaseApi.getInstance().create(LoginApi::class.java)

        val pref = applicationContext.getSharedPreferences("token", Context.MODE_PRIVATE)
        val token = pref.getString("token", null)

        val requestBody = mapOf(
            "storeId" to storeId,
            "roomNumber" to roomNum,
            "fcmToken" to token
        )

        client.sendStoreInfo(requestBody).enqueue(object : Callback<SuccessResponse> {
            override fun onResponse(call : Call<SuccessResponse>, response: Response<SuccessResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null && responseBody.success) {
                        Log.d("Login", "login success")
                        saveStoreInfo(storeId, roomNum)
                    } else {
                        Log.d("Login", "login failed $responseBody")
                        showFailure(this@LoginActivity, "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Login", "request failed $response")
                    showFailure(this@LoginActivity, "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<SuccessResponse>, t: Throwable) {
                Log.e("Login", "$t")
                showFailure(this@LoginActivity, "요청에 실패했습니다.")
            }
        })
    }

    private fun saveStoreInfo(storeId: String, roomNum: String) {
        val sharedPreferences = getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("storeId", storeId)
        editor.putString("roomNum", roomNum)
        editor.apply()

        val intent = Intent(this, StartingActivity::class.java)
        startActivity(intent)
        finish()
    }

    private fun saveManagerInfo(storeId: String) {
        val sharedPreferences = getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("isManager", storeId)
        editor.apply()

        val intent = Intent(this, Manager::class.java)
        startActivity(intent)
        finish()
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

    private fun ManagerLogin(storeId: String) {
        val client = BaseApi.getInstance().create(LoginApi::class.java)

        val pref = applicationContext.getSharedPreferences("token", Context.MODE_PRIVATE)
        val token = pref.getString("token", null)

        val requestBody = mapOf(
            "storeId" to storeId,
            "roomNumber" to "0",
            "fcmToken" to token
        )

        client.sendStoreInfo(requestBody).enqueue(object : Callback<SuccessResponse> {
            override fun onResponse(call : Call<SuccessResponse>, response: Response<SuccessResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null && responseBody.success) {
                        Log.d("Login", "login success")
                        saveManagerInfo(storeId)
                    } else {
                        Log.d("Login", "login failed $responseBody")
                        showFailure(this@LoginActivity, "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Login", "request failed $response")
                    showFailure(this@LoginActivity, "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<SuccessResponse>, t: Throwable) {
                Log.e("Login", "$t")
                showFailure(this@LoginActivity, "요청에 실패했습니다.")
            }
        })
    }
}