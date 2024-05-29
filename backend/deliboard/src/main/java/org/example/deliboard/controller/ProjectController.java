package org.example.deliboard.controller;

import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/api")
public class ProjectController {
    @GetMapping("/version")
    public String getVersion(){
        return "1.0.0";
    }
}
