package com.example.deliboard.beverage.cart

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class CartViewModel : ViewModel() {
    private val _cartItems = MutableLiveData<List<CartItemClass>>()
    val cartItems: LiveData<List<CartItemClass>> = _cartItems

    fun updateCartItems(newItems: List<CartItemClass>) {
        _cartItems.value = newItems
    }
}