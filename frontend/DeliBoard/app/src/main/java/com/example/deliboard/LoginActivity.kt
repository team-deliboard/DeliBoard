package com.example.deliboard

import android.content.Context
import android.content.Intent
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import androidx.appcompat.app.AlertDialog
import androidx.constraintlayout.widget.ConstraintLayout
import com.google.android.material.textfield.TextInputEditText
import retrofit2.Call
import retrofit2.Response
import retrofit2.Callback

class LoginActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_login)

        val loginButton = findViewById<ConstraintLayout>(R.id.loginButton)
        val storeNumInput = findViewById<TextInputEditText>(R.id.storeNumField)
        val tableNumInput = findViewById<TextInputEditText>(R.id.tableNumField)

        loginButton.setOnClickListener {
            val storeNum = storeNumInput.text.toString()
            val tableNum = tableNumInput.text.toString()

            StoreNumCheck(storeNum, tableNum)
        }
    }

    private fun StoreNumCheck(storeNum: String, tableNum: String) {
        val client = BaseApi.getInstance().create(LoginApi::class.java)

        val requestBody = mapOf(
            "storeNum" to storeNum,
            "tableNum" to tableNum
        )

        client.sendStoreInfo(requestBody).enqueue(object : Callback<Void> {
            override fun onResponse(call : Call<Void>, response: Response<Void>) {
                if (response.isSuccessful) {
                    saveStoreInfo(storeNum, tableNum)
                } else {
                    showFailure(this@LoginActivity)
                }
            }

            override fun onFailure(call: Call<Void>, t: Throwable) {
                showFailure(this@LoginActivity)
            }
        })
    }

    private fun saveStoreInfo(storeNum: String, tableNum: String) {
        val sharedPreferences = getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("storeNum", storeNum)
        editor.putString("tableNum", tableNum)
        editor.apply()

        val intent = Intent(this, MainActivity::class.java)
        startActivity(intent)
        finish()
    }

    private fun showFailure(context: Context) {
        val builder = AlertDialog.Builder(context)
        builder.setTitle("인증 실패")
        builder.setMessage("매장 번호, 테이블번호가 유효하지 않습니다.")

        builder.setPositiveButton("확인") {dialog, _ ->
            dialog.dismiss()
        }

        val dialog = builder.create()
        dialog.show()
    }
}