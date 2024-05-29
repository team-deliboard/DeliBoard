package com.example.deliboard.beverage.menus

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.navigation.findNavController
import androidx.recyclerview.widget.GridLayoutManager
import com.example.deliboard.R
import com.example.deliboard.beverage.BeverageAdapter
import com.example.deliboard.beverage.cart.CartAdapter
import com.example.deliboard.beverage.SnackViewModel
import com.example.deliboard.databinding.FragmentSnackBinding
import kotlinx.coroutines.launch

class Snack : Fragment() {

    private lateinit var binding: FragmentSnackBinding

    private val viewModel: SnackViewModel by viewModels()

    private lateinit var cartAdapter: CartAdapter
    private lateinit var beverageAdapter: BeverageAdapter
    private lateinit var snackList: List<MenuClass>

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        // Inflate the layout for this fragment
        binding = FragmentSnackBinding.inflate(layoutInflater)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        cartAdapter = CartAdapter()
        beverageAdapter = BeverageAdapter(requireContext(), emptyList(), cartAdapter) // 초기에 빈 리스트로 어댑터 생성
        binding.snackRV.adapter = beverageAdapter
        binding.snackRV.layoutManager = GridLayoutManager(requireContext(), 4)

        viewModel.getCurrentMenuList()

        lifecycleScope.launch {
            viewModel.snackMenuList.observe(viewLifecycleOwner) { menuList ->
                beverageAdapter.updateData(menuList) // 데이터 업데이트
                snackList = menuList
            }
        }

        // to non-coffee 버튼
        binding.buttonNoCafe.setOnClickListener{
            it.findNavController().navigate(R.id.action_snack_to_noCafe)
        }

        // to snack 버튼
        binding.buttonCoffee.setOnClickListener{
            it.findNavController().navigate(R.id.action_snack_to_coffee)
        }
    }
}