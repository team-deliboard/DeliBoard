package com.example.deliboard.beverage

import android.content.Context
import android.content.SharedPreferences
import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.lifecycle.Observer
import androidx.lifecycle.ViewModelProvider
import androidx.navigation.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.deliboard.BaseApi
import com.example.deliboard.R
import com.example.deliboard.SuccessResponse
import com.example.deliboard.beverage.cart.Cart
import com.example.deliboard.beverage.cart.CartAdapter
import com.example.deliboard.beverage.cart.CartApi
import com.example.deliboard.beverage.cart.CartItemClass
import com.example.deliboard.beverage.cart.CartUpdateListener
import com.example.deliboard.beverage.cart.CartViewModel
import com.example.deliboard.beverage.cart.MenuOrderRequest
import com.example.deliboard.beverage.cart.OrderItem
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class Beverage : Fragment(), CartUpdateListener {

    private lateinit var cartRecyclerView: RecyclerView
    private lateinit var cartAdapter: CartAdapter
    private lateinit var cartViewModel: CartViewModel
    private lateinit var sharedPreferences: SharedPreferences

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        val view = inflater.inflate(R.layout.fragment_beverage, container, false)

        cartRecyclerView = view.findViewById(R.id.cartRV)
        cartAdapter = CartAdapter()
        cartRecyclerView.layoutManager = LinearLayoutManager(context)
        cartRecyclerView.adapter = cartAdapter

        cartViewModel = ViewModelProvider(this)[CartViewModel::class.java]
        cartViewModel.cartItems.observe(viewLifecycleOwner, Observer { cartItems ->
            cartAdapter.submitList(cartItems)
            updateTotalPrice(cartItems)
        })

        updateCartItems()

        sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val roomLogId = sharedPreferences.getString("roomLogId", "")
        val menuOrder = view.findViewById<TextView>(R.id.orderButton)
        menuOrder.setOnClickListener{
            val cartItems = Cart.getInstance().getItems()
            sendOrder(roomLogId!!, cartItems)
//            lifecycleScope.launch {
//                val cartItems = Cart.getInstance().getItems()
//                sendOrder(roomLogId!!, cartItems)
//            }
        }

        val delCart = view.findViewById<TextView>(R.id.cartDeleteButton)
        delCart.setOnClickListener{
            ClearCartButton()
        }

        return view
    }

    private fun updateTotalPrice(cartItems: List<CartItemClass>) {
        val totalPriceTextView = view?.findViewById<TextView>(R.id.cartPrice)
        val totalPrice = cartItems.sumBy { it.price * it.quantity }
        totalPriceTextView?.text = "$totalPrice 원"
    }

    private fun updateCartItems() {
        // 장바구니에 담긴 아이템 리스트를 가져옵니다.
        val cartItems = Cart.getInstance().getItems()

        // Adapter에 아이템 리스트 업데이트
//        cartAdapter.submitList(cartItems)
        cartViewModel.updateCartItems(cartItems)

        val totalPriceTextView = view?.findViewById<TextView>(R.id.cartPrice)
        val totalPrice = cartAdapter.calculateTotalPrice()
        totalPriceTextView?.text = "$totalPrice 원"
    }

    private fun sendOrder(roomLogId: String, cartItems: List<CartItemClass>) {
        val client = BaseApi.getInstance().create(CartApi::class.java)

        val orderItems = cartItems.map { OrderItem(it.id, it.quantity) }
        val requestBody = MenuOrderRequest(roomLogId, orderItems)

        client.MenuOrder(requestBody).enqueue(object : Callback<SuccessResponse> {
            override fun onResponse(call : Call<SuccessResponse>, response: Response<SuccessResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null && responseBody.success) {
                        showOrderedDialog()
//                        Log.d("menuOrder", "Menu order success")
                    } else {
                        Log.d("menuOrder", "Menu order failed")
                    }
                } else {
                    Log.d("menuOrder", "request failed")
                }
            }

            override fun onFailure(call: Call<SuccessResponse>, t: Throwable) {
                Log.e("Theme", "$t")
            }
        })
    }

    private fun showOrderedDialog() {
        val alertDialogBuilder = AlertDialog.Builder(requireContext())
        alertDialogBuilder.apply {
            setTitle("주문완료")
            setMessage("주문이 완료되었습니다.\n잠시만 기다려주세요")
            setPositiveButton("확인") {dialog, _ ->
                dialog.dismiss()
                ClearCartButton()
                requireView().findNavController().navigate(R.id.action_beverage_to_Home)
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    override fun onCartUpdated() {
        cartViewModel.updateCartItems(Cart.getInstance().getItems())
    }

    private fun ClearCartButton() {
        Cart.getInstance().clearCart()
        (requireActivity() as? CartUpdateListener)?.onCartUpdated()
    }
}