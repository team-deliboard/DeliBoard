package org.example.deliboard.controller;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.StockDtoDetail;
import org.example.deliboard.model.Store;
import org.example.deliboard.repository.StoreRepository;
import org.example.deliboard.service.StockService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.Collections;
import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/stock")
public class StockController {
    private final StockService stockService;
    private final StoreRepository storeRepository;

    @GetMapping("/games")
    public List<StockDtoDetail> getStocksByStoreId(@RequestParam Integer storeId) {
        Store store = storeRepository.findStoreById(storeId).orElse(null);
        if (store == null)
            return Collections.emptyList();
        return stockService.getStockInStore(store);
    }
}
