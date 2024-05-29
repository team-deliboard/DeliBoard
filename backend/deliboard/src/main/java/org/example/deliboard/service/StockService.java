package org.example.deliboard.service;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.StockDtoDetail;
import org.example.deliboard.model.Game;
import org.example.deliboard.model.Stock;
import org.example.deliboard.model.Store;
import org.example.deliboard.repository.*;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;

@Service
@Transactional
@RequiredArgsConstructor
public class StockService {
    static final String SERVER_URL = System.getenv("SERVER_URL");
    private final StockRepository stockRepository;
    private final GameRepository gameRepository;

    private StockDtoDetail convertToDto(Game game, Integer stockCount) {
        String thumbnailUrl = SERVER_URL + "/image/game/" + game.getBggId();
        return StockDtoDetail.builder()
                .id(game.getId())
                .title(game.getTitle())
                .description(game.getDetail())
                .thumbnailUrl(thumbnailUrl)
                .stock(stockCount)
                .minPlayer(game.getMinPlayer())
                .maxPlayer(game.getMaxPlayer())
                .minPlaytime(game.getMinPlaytime())
                .maxPlaytime(game.getMaxPlaytime())
                .difficulty(game.getDifficulty())
                .theme(game.getTag())
                .thumbnailUrl(thumbnailUrl)
                .build();
    }

    public List<StockDtoDetail> getStockInStore(Store store) {
        Map<Integer, Integer> stockCount = countStocksByGameId(store);
        List<StockDtoDetail> response = new ArrayList<>();

        for (Integer key : stockCount.keySet()) {
            Game game = gameRepository.findGameById(key).orElse(null);
            if (game == null) continue;
            response.add(convertToDto(game, stockCount.get(key)));
        }
        return response;
    }

    public Map<Integer, Integer> countStocksByGameId(Store store) {
        List<Stock> buf = stockRepository.findByStoreId(store.getId());
        Map<Integer, Integer> stockCount = new HashMap<>();
        for (Stock stock : buf) {
            Integer gameId = stock.getGame().getId();
            if (stockCount.containsKey(gameId)) {
                if (!stock.getIsAvailable()) continue;
                stockCount.put(gameId, stockCount.get(gameId) + 1);
            } else {
                if (stock.getIsAvailable()) {
                    stockCount.put(gameId, 1);
                } else {
                    stockCount.put(gameId, 0);
                }
            }
        }
        return stockCount;
    }
}
