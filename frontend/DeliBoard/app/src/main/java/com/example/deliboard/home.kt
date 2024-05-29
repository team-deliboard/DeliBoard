package com.example.deliboard

import android.os.Bundle
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.navigation.findNavController


class home : Fragment() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        val view = inflater.inflate(R.layout.fragment_home, container, false)

        view.findViewById<ConstraintLayout>(R.id.button1).setOnClickListener{
            it.findNavController().navigate(R.id.action_Home_to_GameList)
        }

        view.findViewById<ConstraintLayout>(R.id.button2).setOnClickListener{
            it.findNavController().navigate(R.id.action_Home_to_recommend)
        }

        view.findViewById<ConstraintLayout>(R.id.button3).setOnClickListener{
            it.findNavController().navigate(R.id.action_Home_to_beverage)
        }

        view.findViewById<ConstraintLayout>(R.id.button4).setOnClickListener{

        }
        view.findViewById<ConstraintLayout>(R.id.button5).setOnClickListener{

        }
        view.findViewById<ConstraintLayout>(R.id.button6).setOnClickListener{

        }
        view.findViewById<ConstraintLayout>(R.id.button7).setOnClickListener{

        }
        return view
    }
}