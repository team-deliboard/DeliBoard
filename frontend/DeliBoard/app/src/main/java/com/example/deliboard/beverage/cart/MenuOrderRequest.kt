package com.example.deliboard.beverage.cart

data class MenuOrderRequest(
    val roomLogId: String,
    val itemList: List<OrderItem>
)

data class OrderItem(
    val menuId: Int,
    val quantity: Int
)
