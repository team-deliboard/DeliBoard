package org.example.deliboard.model;

import jakarta.persistence.*;
import lombok.Data;
//import org.springframework.cglib.core.Local;

import java.time.LocalDateTime;

// 식음료 주문 내역
@Entity(name = "order_menu")
@Data
public class MenuOrder {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(nullable = false)
    private Long id;

    @Column(nullable = false)
    private Boolean isDelivered;

    @ManyToOne
    @JoinColumn(name="room_log_id")
    private RoomLog roomLog;

    @ManyToOne
    @JoinColumn(name="store_id")
    private Store store;

    //테이블 ID
    @ManyToOne
    @JoinColumn(name="room_id")
    private Room room;

    //메뉴 ID
    @ManyToOne
    @JoinColumn(name="menu_id")
    private Menu menu;

    @Column(nullable = false)
    private Integer quantity;

    @Column(nullable = false)
    private LocalDateTime createdDttm;

    @Column
    private LocalDateTime deliveredDttm;

    @PrePersist
    public void prePersist() {
        createdDttm = LocalDateTime.now();
        isDelivered = false;
    }
}
