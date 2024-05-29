package com.example.deliboard.manager

import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.widget.Button
import android.widget.TextView
import androidx.activity.enableEdgeToEdge
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.lifecycle.Observer
import androidx.lifecycle.ViewModelProvider
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.deliboard.R
import com.example.deliboard.databinding.ActivityManagerBinding
import com.example.deliboard.databinding.FragmentGameListBinding
import com.example.deliboard.login.LoginActivity

class Manager : AppCompatActivity() {
    private lateinit var manageViewModel: ManageViewModel

    private lateinit var manageAdapter: ManageAdapter
    private lateinit var orderList: List<ManageClass>

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContentView(R.layout.activity_manager)

        manageViewModel = ViewModelProvider(this).get(ManageViewModel::class.java)

        manageAdapter = ManageAdapter(emptyList())

        val manageRV = this.findViewById<RecyclerView>(R.id.managerRV)
        manageRV.adapter = manageAdapter
        manageRV.layoutManager = LinearLayoutManager(this)

        manageViewModel.getCurrentOrderList()

        val titleButton = this.findViewById<TextView>(R.id.hostTitle)
        titleButton.setOnClickListener{
            val intent = Intent(this@Manager, Manager::class.java)
            intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP or Intent.FLAG_ACTIVITY_NEW_TASK)
            startActivity(intent)
            finish()
        }

        val quitButton = this.findViewById<Button>(R.id.managerQuitButton)
        quitButton.setOnClickListener{
            showLogoutDialog()
        }


        // 필요한 경우 뷰모델에 데이터를 관찰하고 업데이트할 수 있습니다.
        manageViewModel.orderList.observe(this, Observer { data ->
            manageAdapter.updateData(data)
            orderList = data
        })
    }

    private fun showLogoutDialog() {
        val alertDialogBuilder = AlertDialog.Builder(this)
        alertDialogBuilder.apply {
            setTitle("연결해제")
            setMessage("현재 키오스크를 연결해제 합니다.")
            setPositiveButton("연결해제") {_, _ ->
                val sharedPreferences = getSharedPreferences("StoreInfo", Context.MODE_PRIVATE)

                sharedPreferences.edit().remove("isManager").apply()

                val intent = Intent(this@Manager, LoginActivity::class.java)
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
}