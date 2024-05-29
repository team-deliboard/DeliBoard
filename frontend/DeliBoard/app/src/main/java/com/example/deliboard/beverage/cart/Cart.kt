package com.example.deliboard.beverage.cart

class Cart private constructor() {
    private val items: MutableList<CartItemClass> = mutableListOf()

    companion object {
        @Volatile
        private var instance: Cart? = null

        fun getInstance(): Cart {
            return instance ?: synchronized(this) {
                instance ?: Cart().also { instance = it }
            }
        }
    }

    fun getItems(): List<CartItemClass> {
        return items
    }

    fun addItem(item: CartItemClass) {
        val existingItem = items.find { it.id == item.id }
        if (existingItem != null) {
            // 이미 장바구니에 있는 아이템인 경우 수량을 증가시킵니다.
            existingItem.quantity++
        } else {
            // 장바구니에 없는 아이템인 경우 새로 추가합니다.
            items.add(item)
        }
    }

    fun removeItem(item: CartItemClass) {
        items.remove(item)
    }

    fun clearCart() {
        items.clear()
    }
}