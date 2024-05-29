package com.example.deliboard.beverage

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.deliboard.beverage.menus.MenuClass
import kotlinx.coroutines.launch


class CoffeeViewModel : ViewModel()  {
    private val beverageNetwork = BeverageNetwork()

    private val _menuList = MutableLiveData<List<MenuClass>>()
    val coffeeMenuList: LiveData<List<MenuClass>> = _menuList

    fun getCurrentMenuList() {
        viewModelScope.launch {
            try {
                val result = beverageNetwork.getMenuList()
                val coffeeMenu = result.filter { it.type == 0 }
                _menuList.value = coffeeMenu
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}

class NonCoffeeViewModel : ViewModel()  {
    private val beverageNetwork = BeverageNetwork()

    private val _menuList = MutableLiveData<List<MenuClass>>()
    val nonCoffeeMenuList: LiveData<List<MenuClass>> = _menuList

    fun getCurrentMenuList() {
        viewModelScope.launch {
            try {
                val result = beverageNetwork.getMenuList()
                val nonCoffeeMenu = result.filter { it.type == 1 }
                _menuList.value = nonCoffeeMenu
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}

class SnackViewModel : ViewModel()  {
    private val beverageNetwork = BeverageNetwork()

    private val _menuList = MutableLiveData<List<MenuClass>>()
    val snackMenuList: LiveData<List<MenuClass>> = _menuList

    fun getCurrentMenuList() {
        viewModelScope.launch {
            try {
                val result = beverageNetwork.getMenuList()
                val snackMenu = result.filter { it.type == 2 }
                _menuList.value = snackMenu
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}