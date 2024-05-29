package com.example.deliboard.recommend

import com.example.deliboard.game.GameClass
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import com.example.deliboard.R
import com.example.deliboard.databinding.FragmentRecommendBinding
import com.example.deliboard.game.GameNetwork

class Recommend : Fragment() {

    private lateinit var binding: FragmentRecommendBinding

    private lateinit var recommendAdapter: RecommendAdapter
    private lateinit var gameList: List<GameClass>

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        val view = inflater.inflate(R.layout.fragment_recommend, container, false)


        return view
    }

    private suspend fun getRandomGame(): GameClass? {
        val gameNetwork = GameNetwork()
        val gameList = gameNetwork.getGameList()

        return if (gameList.isNotEmpty()) {
            val randomIndex = (0 until gameList.size).random()
            gameList[randomIndex]
        } else {
            null
        }
    }
}