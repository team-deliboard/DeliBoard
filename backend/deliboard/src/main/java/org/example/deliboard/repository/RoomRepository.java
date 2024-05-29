package org.example.deliboard.repository;

import org.example.deliboard.model.Room;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;
import org.springframework.stereotype.Repository;
import org.springframework.transaction.annotation.Transactional;

import java.util.Optional;

@Repository
public interface RoomRepository extends JpaRepository<Room, Integer> {
    Optional<Room> findRoomById(Integer id);
    Optional<Room> findRoomByNumberAndStoreId(Byte roomNumber, Integer storeId);
    Optional<Room> findRoomByStoreIdAndNumber(Integer storeId, Byte roomNumber);

    @Modifying
    @Transactional
    @Query("update room r set r.fcmToken = ?2 where r.id = ?1")
    void updateRoomToken(Integer roomId, String fcmToken);
}
