package com.example.deliboard.manager

import android.graphics.Color
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.deliboard.BaseApi
import com.example.deliboard.R
import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class ManageAdapter(private var dataList: List<ManageClass>) :
    RecyclerView.Adapter<ManageAdapter.ManageViewHolder>() {



    // ViewHolder 클래스 정의
    inner class ManageViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView) {
        val nameTextView: TextView = itemView.findViewById(R.id.hostMenuName)
        val quanTextView: TextView = itemView.findViewById(R.id.hostMenuQuan)
        val comButton: Button = itemView.findViewById(R.id.hostMenuButton)
        val callButton: Button = itemView.findViewById(R.id.hostCallTurtle)

        init {
            comButton.setOnClickListener{
                val order = dataList[adapterPosition]
                val orderId = order.id
                sendComplete(orderId)
            }

            callButton.setOnClickListener{
                val order = dataList[adapterPosition]
                val roomLogId = order.roomLogId
                callTurtleBot(roomLogId)
            }
        }
    }

    // onCreateViewHolder: ViewHolder 객체를 생성하고 뷰를 연결
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ManageViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.item_layout_host, parent, false)
        return ManageViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: ManageViewHolder, position: Int) {
        val menuItem = dataList[position]
//            println(gameItem.thumbnailUrl)
        if (menuItem.isDelivered == true) {
            holder.itemView.setBackgroundColor(Color.LTGRAY)
            holder.quanTextView.setTextColor(Color.GRAY)
            holder.comButton.setTextColor(Color.GRAY)
            holder.callButton.setTextColor(Color.GRAY)
        }
        holder.nameTextView.text = menuItem.menuName
        holder.quanTextView.text = "${menuItem.quantity} 개"
    }

    // getItemCount: 데이터 목록의 크기 반환
    override fun getItemCount() = dataList.size

    fun updateData(newOrderList: List<ManageClass>) {
        dataList = newOrderList
        notifyDataSetChanged() // 변경 사항을 RecyclerView에 알림
    }

    private fun callTurtleBot(roomLogId: String) {
        val service = BaseApi.getInstance().create((ManageApi::class.java))

        val call: Call<SuccessResponse> = service.callTurtle(roomLogId)
//        println("터틀봇 호출 눌러짐")

        call.enqueue(object : Callback<SuccessResponse> {
            override  fun onResponse(call: Call<SuccessResponse>, response: Response<SuccessResponse>) {
                if (response.isSuccessful) {
                    val responseResult: SuccessResponse? = response.body()
                    val success: Boolean? = responseResult?.success
                    val message: String? = responseResult?.message

                    // 성공 여부에 따른 로직 처리
                    if (success == true) {
                        Log.d("Order", "Success: $message")
                    } else {
                        // 실패 처리
                        Log.d("Order", "Failed: $message")
                    }
                } else {
                    Log.d("Order", "Fail to send: ${response.errorBody()}")
                }
            }
            override fun onFailure(call: Call<SuccessResponse>, t:Throwable) {
                Log.d("Order", "$t")
            }
        })
    }

    private fun sendComplete(orderId: Int) {
        val service = BaseApi.getInstance().create((ManageApi::class.java))

        val requestBody = mapOf(
            "id" to orderId
        )
//        println("완료메세지 눌러짐")

        service.sendMenuDeliverded(requestBody).enqueue(object : Callback<SuccessResponse> {
            override fun onResponse(call : Call<SuccessResponse>, response: Response<SuccessResponse>) {
                if (response.isSuccessful) {
                    Log.d("MenuComplete", "Success : $response")
                } else {
                    Log.d("MenuComplete", "$response")
                }
            }
            override fun onFailure(call: Call<SuccessResponse>, t:Throwable) {
                Log.d("MenuComplete", "$t")
            }
        })
    }
}