package org.example.deliboard.model;

import jakarta.persistence.*;
import lombok.Data;
//import org.springframework.cglib.core.Local;

//import javax.swing.text.StyledEditorKit;
import java.time.LocalDateTime;

// 게임 주문 내역
@Entity(name = "game_order")
@Data
public class GameOrder {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(nullable = false)
    private Long id;

    @Column(nullable = false)
    private Boolean isDelivered;

    //테이블 ID
    @ManyToOne
    @JoinColumn(name="room_id")
    private Room room;

    //게임 ID
    @ManyToOne
    @JoinColumn(name="stock_id")
    private Game stock;

    // 주문 일시
    @Column(nullable = false)
    private LocalDateTime createdDttm;

    // 완료 일시
    @Column
    private LocalDateTime deliveredDttm;

    // 타입 : 0: 주문/ 1: 회수
    @Column(nullable = false)
    private Boolean type;

    @PrePersist
    public void prePersist() {
        createdDttm = LocalDateTime.now();
        isDelivered = false;
    }
}
