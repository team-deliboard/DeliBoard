package com.example.deliboard.manager

import com.example.deliboard.BaseApi

class ManageNetwork {
    private val client = BaseApi.getInstance().create(ManageApi::class.java)

    suspend fun getOrderList() = client.getOrderList("1");
}