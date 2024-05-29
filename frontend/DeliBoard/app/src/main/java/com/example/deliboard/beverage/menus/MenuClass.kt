package com.example.deliboard.beverage.menus

data class MenuClass(
    val id: Int,
    val name: String,
    val type: Int,
    val price: Int,
//    val imageURL: String,
    val isAvailable: Boolean?,
    val storeId: Int,
)
