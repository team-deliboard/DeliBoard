package org.example.deliboard.repository;

import org.example.deliboard.model.RoomLog;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.Optional;

@Repository
public interface RoomLogRepository extends JpaRepository<RoomLog, Integer> {
    Optional<RoomLog> findRoomLogById(Integer roomId);
    Optional<RoomLog> findRoomLogByIdAndIsCheckedOut(Integer roomLogId, Boolean isCheckedOut);
}
