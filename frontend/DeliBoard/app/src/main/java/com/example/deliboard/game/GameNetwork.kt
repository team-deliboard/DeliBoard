package com.example.deliboard.game

import com.example.deliboard.BaseApi

class GameNetwork {

    private val client = BaseApi.getInstance().create(GameApi::class.java)

    suspend fun getGameList() = client.getGameList("1");
}