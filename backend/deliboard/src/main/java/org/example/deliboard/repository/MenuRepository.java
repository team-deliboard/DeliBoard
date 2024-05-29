package org.example.deliboard.repository;

import org.example.deliboard.model.Menu;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;

@Repository
public interface MenuRepository extends JpaRepository<Menu, Integer> {
    List<Menu> findMenuByStoreId(Integer storeId);
    Optional<Menu> findMenuById(Integer id);
}
