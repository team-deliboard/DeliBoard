package org.example.deliboard.model;

import jakarta.persistence.*;
import lombok.Data;

@Entity(name = "room")
@Data
public class Room {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(nullable = false)
    private Integer id;

    // 테이블 좌표
    @Column(nullable = false)
    private String location;

    // 매장 ID
    @ManyToOne
    @JoinColumn(name="store_id")
    private Store store;

    // 테이블 번호
    @Column(nullable = false)
    private Byte number;

    @Column
    private String fcmToken;
}
