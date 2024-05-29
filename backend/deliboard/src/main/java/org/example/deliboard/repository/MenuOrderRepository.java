package org.example.deliboard.repository;

import org.example.deliboard.model.MenuOrder;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;

@Repository
public interface MenuOrderRepository extends JpaRepository<MenuOrder, Integer> {
    List<MenuOrder> findMenuOrderByIsDeliveredAndStoreId(boolean isDelivered, Integer storeId);
    List<MenuOrder> findMenuOrderByRoomLogId(Integer roomLogId);
    Optional<MenuOrder> findMenuOrderById(Long menuOrderId);
}
