package org.example.deliboard.model;

import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;

// 보드게임 재고
@Entity(name = "stock")
@Getter @Setter @EqualsAndHashCode @AllArgsConstructor @NoArgsConstructor
public class Stock {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Integer id;

    // 재고 여부
    @Column(nullable = false)
    private Boolean isAvailable;

    // 등록 일시
    @Column(nullable = false)
    private LocalDateTime createdDttm;

    // 매장 ID
    @ManyToOne
    @JoinColumn(name="store_id")
    private Store store;

    // 게임 ID
    @ManyToOne
    @JoinColumn(name="game_id")
    private Game game;

    // 재고의 좌표
    @Column(name = "location_x")
    private String locationX;

    @Column(name = "location_y")
    private String locationY;


}
