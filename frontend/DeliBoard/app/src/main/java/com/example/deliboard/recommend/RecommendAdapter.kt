package com.example.deliboard.recommend

import com.example.deliboard.game.GameClass
import android.content.Context
import android.graphics.Color
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.navigation.findNavController
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.deliboard.R
import kotlin.math.ceil

class RecommendAdapter (private val context: Context, private var dataList: List<GameClass>) :
    RecyclerView.Adapter<RecommendAdapter.RecommendViewHolder>() {

    private var filteredList : List<GameClass> = dataList

    inner class RecommendViewHolder(itemView: View) : RecyclerView.ViewHolder(itemView), View.OnClickListener {
        val titleTextView: TextView = itemView.findViewById(R.id.listTitle)
        val descriptionTextView: TextView = itemView.findViewById(R.id.listDescription)
        val stockTextView: TextView = itemView.findViewById(R.id.listStock)
        val playerTextView: TextView = itemView.findViewById(R.id.listPlayer)
        //            val maxPlayerTextView: TextView = itemView.findViewById(R.id.maxPlayer)
        val playTimeTextView: TextView = itemView.findViewById(R.id.listPlaytime)
        //            val maxPlayTimeTextView: TextView = itemView.findViewById(R.id.maxPlaytime)
        val difficultyTextView: TextView = itemView.findViewById(R.id.listDifficulty)
        val themeTextView: TextView = itemView.findViewById(R.id.listTheme)
        val imageView: ImageView = itemView.findViewById(R.id.thumbnailImage)

        init {
            itemView.setOnClickListener(this)
        }

        override fun onClick(v: View?) {
            val game = dataList[adapterPosition]
            if (game.stock > 0) {
                val bundle = Bundle().apply {
                    putString("gameId", game.id.toString())
                    putString("title", game.title)
                    putString("description", game.description)
                    putString("thumbnailUrl", game.thumbnailUrl)
                    putString("stock", game.stock.toString())
                    putString("minPlayer", game.minPlayer.toString())
                    putString("maxPlayer", game.maxPlayer.toString())
                    putString("minPlaytime", game.minPlaytime.toString())
                    putString("maxPlaytime", game.maxPlaytime.toString())
                    putString("difficulty", ceil(game.difficulty).toInt().toString())
                    putString("theme", game.theme)
                }
                v?.findNavController()?.navigate(R.id.action_GameList_to_gameDetail, bundle)
            } else {
                Log.e("List", "e")
            }
        }
    }

    // onCreateViewHolder: ViewHolder 객체를 생성하고 뷰를 연결
    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): RecommendViewHolder {
        val view = LayoutInflater.from(parent.context).inflate(R.layout.item_layout_game, parent, false)
        return RecommendViewHolder(view)
    }

    // onBindViewHolder: 데이터를 뷰에 연결
    override fun onBindViewHolder(holder: RecommendViewHolder, position: Int) {
        val gameItem = dataList[position]
//            println(gameItem.thumbnailUrl)
        if (gameItem.stock == 0) {
            holder.itemView.setBackgroundColor(Color.LTGRAY)
            holder.titleTextView.setTextColor(Color.GRAY)
            holder.descriptionTextView.setTextColor(Color.GRAY)
            holder.playerTextView.setTextColor(Color.GRAY)
            holder.playTimeTextView.setTextColor(Color.GRAY)
            holder.difficultyTextView.setTextColor(Color.GRAY)
            holder.stockTextView.setTextColor(Color.GRAY)
            holder.themeTextView.setTextColor(Color.GRAY)
        } else {
            holder.itemView.setBackgroundColor(Color.WHITE)
        }
        holder.titleTextView.text = gameItem.title
        holder.descriptionTextView.text = if (gameItem.description.length > 30) {
            gameItem.description.substring(0, 30) + "..."
        } else {
            gameItem.description
        }
        holder.stockTextView.text = if (gameItem.stock > 0) {
            "재고 : ${gameItem.stock}"
        } else {
            "재고 : X"
        }
        holder.playerTextView.text = if (gameItem.minPlayer == gameItem.maxPlayer) {
            "인원 : ${gameItem.minPlayer}명, "
        } else {
            "인원 : ${gameItem.minPlayer} ~ ${gameItem.maxPlayer}명, "
        }
//            holder.maxPlayerTextView.text = gameItem.maxPlayer.toString()
        holder.playTimeTextView.text = if (gameItem.minPlaytime == gameItem.maxPlaytime) {
            "시간 : ${gameItem.minPlaytime}분, "
        } else {
            "시간 : ${gameItem.minPlaytime} ~ ${gameItem.maxPlaytime}분, "
        }
//            holder.maxPlayTimeTextView.text = gameItem.maxPlaytime.toString()
        holder.difficultyTextView.text = "난이도 : ${"★".repeat(ceil(gameItem.difficulty).toInt())}"
        holder.themeTextView.text = "장르 : ${gameItem.theme}, "
        Glide.with(context)
            .load(gameItem.thumbnailUrl)
            .placeholder(R.drawable.ipad) // 로딩이미지
            .error(R.drawable.chess_black) //실패이미지
            .into(holder.imageView)
    }

    // getItemCount: 데이터 목록의 크기 반환
    override fun getItemCount() = dataList.size

    fun updateData(newGameList: List<GameClass>) {
        dataList = newGameList
        notifyDataSetChanged() // 변경 사항을 RecyclerView에 알림
    }

}