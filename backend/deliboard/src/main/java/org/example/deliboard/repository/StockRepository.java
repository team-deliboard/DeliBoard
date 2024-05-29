package org.example.deliboard.repository;

import jakarta.transaction.Transactional;
import org.example.deliboard.model.Stock;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;

@Repository
public interface StockRepository extends JpaRepository<Stock, Integer> {
    Optional<Stock> findStockById(Integer id);
    List<Stock> findByStoreId(Integer storeId);
    List<Stock> findByGameIdAndStoreId(Integer gameId, Integer storeId);
    List<Stock> findByGameId(Integer gameId);
    List<Stock> findByGameIdAndIsAvailable(Integer gameId, Boolean isAvailable);

    @Modifying
    @Transactional
    @Query("update stock s set s.isAvailable = ?2 where s.id = ?1")
    void updateStockStatus(Integer stockId, Boolean isAvailable);
}
