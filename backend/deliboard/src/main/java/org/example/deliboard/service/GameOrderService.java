package org.example.deliboard.service;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.GeneralResponseDto;
import org.example.deliboard.model.*;
import org.example.deliboard.repository.RoomLogRepository;
import org.example.deliboard.repository.RoomRepository;
import org.example.deliboard.repository.StockRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.List;
import java.util.concurrent.CompletableFuture;

@Service
@Transactional
@RequiredArgsConstructor
public class GameOrderService {
    private final StockRepository stockRepository;
    private final SocketClientService socketClientService;
    private final RoomRepository roomRepository;
    private final NotificationService notificationService;
    private final RoomLogRepository roomLogRepository;

    public void changeRoomStock(RoomLog roomLog, Stock stock) {
        roomLog.setStock(stock);
        roomLogRepository.save(roomLog);
    }

    @Async
    public CompletableFuture<GeneralResponseDto> orderGame(Game game, RoomLog roomLog) {
        List<Stock> stocks = stockRepository.findByGameIdAndIsAvailable(game.getId(), true);
        if (stocks.isEmpty()) {
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, "No stock available"));
        }
        Stock stock = stocks.get(0);
        String socket_response = socketClientService.sendGameOrderToSocket(
                roomLog.getRoom().getStore().getId(),
                roomLog.getRoom().getNumber(),
                stock
        );
        System.out.println(socket_response);
        if (socket_response == null) {
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, "No response from socket server"));
        }
        if (socket_response.equals("Success")) {
//            stock.setIsAvailable(false);
            changeRoomStock(roomLog, stock);
            return CompletableFuture.completedFuture(new GeneralResponseDto(true, "Order placed successfully"));
        } else {
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, socket_response));
        }
    }
    @Async
    public CompletableFuture<GeneralResponseDto> returnGame(RoomLog roomLog) {
        String socket_response = socketClientService.sendGameReturnToSocket(roomLog);
        if (socket_response.equals("Success")) {
//            stockRepository.updateStockStatus(stockId, true);
            changeRoomStock(roomLog, null);
            return CompletableFuture.completedFuture(new GeneralResponseDto(true, "Return placed successfully"));
        } else {
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, socket_response));
        }
    }

    public GeneralResponseDto sendNotification(Room room, String message) {
        return notificationService.sendNotification(room.getFcmToken(), "Say hello to turtle~", message);
    }
}
