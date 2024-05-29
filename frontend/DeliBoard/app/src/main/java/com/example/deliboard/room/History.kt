package com.example.deliboard.room

import android.content.Context
import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.deliboard.databinding.FragmentOrderHistoryBinding
import kotlinx.coroutines.launch

class History : Fragment() {
    private lateinit var binding: FragmentOrderHistoryBinding

    private lateinit var viewModel: HistoryViewModel

    private lateinit var historyAdapter: HistoryAdapter
    private lateinit var historyList: List<HistoryClass>

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        // Inflate the layout for this fragment
        binding = FragmentOrderHistoryBinding.inflate(layoutInflater)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        val sharedPreferences =
            requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        val roomLogId = sharedPreferences.getString("roomLogId", "")

        viewModel = HistoryViewModel(roomLogId!!)
        viewModel.getHistoryList()

        historyAdapter = HistoryAdapter(requireContext(), emptyList()) // 초기에 빈 리스트로 어댑터 생성
        binding.orderHistoryRV.adapter = historyAdapter
        binding.orderHistoryRV.layoutManager = LinearLayoutManager(requireContext())

        val emptyTextView = binding.emptyTextView

        lifecycleScope.launch {
            viewModel.historyList.observe(viewLifecycleOwner) { historylist ->
                println(historylist)

                if (historylist.isEmpty()) {
                    emptyTextView.visibility = View.VISIBLE // 비어 있는 경우 TextView 표시
                } else {
                    emptyTextView.visibility = View.GONE // 비어 있지 않은 경우 TextView 숨김
                    historyAdapter.updateData(historylist) // 데이터 업데이트
                    historyList = historylist
                }
            }
        }
    }
}