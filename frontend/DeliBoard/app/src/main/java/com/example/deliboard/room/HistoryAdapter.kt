package com.example.deliboard.room

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.deliboard.R

class HistoryAdapter(private val context: Context, private var dataList: List<HistoryClass>) :
    RecyclerView.Adapter<HistoryAdapter.HistoryViewHolder>() {

    // ViewHolder 클래스 정의
    inner class HistoryViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val nameTextView: TextView = itemView.findViewById(R.id.historyName)
        val quanTextView: TextView = itemView.findViewById(R.id.historyQuan)

        override fun onClick(v: View?) {
            val history = dataList[adapterPosition]
        }
    }

    // onCreateViewHolder: ViewHolder 객체를 생성하고 뷰를 연결
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): HistoryViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.item_layout_order_history, parent, false)
        return HistoryViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: HistoryViewHolder, position: Int) {
        val historyItem = dataList[position]
        holder.nameTextView.text = historyItem.name
        holder.quanTextView.text = "${historyItem.quantity} 개"
    }

    // getItemCount: 데이터 목록의 크기 반환
    override fun getItemCount() = dataList.size

    fun updateData(newHistoryList: List<HistoryClass>) {
        dataList = newHistoryList
        notifyDataSetChanged() // 변경 사항을 RecyclerView에 알림
    }
}