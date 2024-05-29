package org.example.deliboard.controller;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.*;
import org.example.deliboard.model.Game;
import org.example.deliboard.model.Room;
import org.example.deliboard.model.RoomLog;
import org.example.deliboard.repository.GameRepository;
import org.example.deliboard.repository.RoomLogRepository;
import org.example.deliboard.repository.RoomRepository;
import org.example.deliboard.repository.StoreRepository;
import org.example.deliboard.service.GameOrderService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.concurrent.CompletableFuture;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/game-order")
public class GameOrderController {
    private final GameOrderService gameOrderService;
    private final RoomRepository roomRepository;
    private final GameRepository gameRepository;
    private final StoreRepository storeRepository;
    private final RoomLogRepository roomLogRepository;

    @GetMapping
    public CompletableFuture<GeneralResponseDto> orderGame(@RequestParam Integer roomLogId,
                                                           @RequestParam Integer gameId) {
        Game game = gameRepository.findGameById(gameId).orElse(null);
        RoomLog roomLog = roomLogRepository.findRoomLogByIdAndIsCheckedOut(roomLogId, false).orElse(null);
        if (game == null || roomLog == null)
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, "Invalid request: Cannot find game or room with given parameters"));
        return gameOrderService.orderGame(game, roomLog);
    }

    @GetMapping("/return")
    public CompletableFuture<GeneralResponseDto> returnGame(@RequestParam Integer roomLogId) {
        RoomLog roomLog = roomLogRepository.findRoomLogByIdAndIsCheckedOut(roomLogId, false).orElse(null);
        if (roomLog == null)
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, "Invalid request: Room is empty or checked out already"));
        return gameOrderService.returnGame(roomLog);
    }

    @PostMapping("/notification")
    public GeneralResponseDto sendNotification(@RequestBody RoomMessageDto body) {
        Room room = roomRepository.findRoomByNumberAndStoreId(body.getRoomNumber(), body.getStoreId()).orElse(null);
        if (room == null)
            return new GeneralResponseDto(false, "Room not found");
        return gameOrderService.sendNotification(room, body.getMessage());
    }
}
