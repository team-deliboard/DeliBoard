package com.example.deliboard.beverage.cart

data class CartItemClass(
    val id: Int,
    val name: String,
    val price: Int,
    var quantity: Int = 1 // 수량 필드 추가, 기본값은 1
)