package org.example.deliboard.controller;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.GeneralResponseDto;
import org.example.deliboard.model.Room;
import org.example.deliboard.model.RoomLog;
import org.example.deliboard.repository.RoomLogRepository;
import org.example.deliboard.repository.RoomRepository;
import org.example.deliboard.service.SocketClientService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/turtle")
public class TurtleController {
    private final SocketClientService socketClientService;
    private final RoomLogRepository roomLogRepository;

    @GetMapping("/call")
    public GeneralResponseDto callTurtle(@RequestParam Integer roomLogId) {
        RoomLog roomLog = roomLogRepository.findById(roomLogId).orElse(null);
        if (roomLog == null) {
            return new GeneralResponseDto(false, "RoomLog not found");
        }
        return socketClientService.callTurtle(roomLog);
    }
}
