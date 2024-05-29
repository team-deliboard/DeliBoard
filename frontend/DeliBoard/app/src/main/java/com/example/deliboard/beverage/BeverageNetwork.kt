package com.example.deliboard.beverage

import com.example.deliboard.BaseApi

class BeverageNetwork {

    private val client = BaseApi.getInstance().create(BeverageApi::class.java)

    suspend fun getMenuList() = client.getMenuList("1")
}