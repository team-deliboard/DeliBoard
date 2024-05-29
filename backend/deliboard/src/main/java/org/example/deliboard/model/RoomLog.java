package org.example.deliboard.model;

import jakarta.persistence.*;
import lombok.Data;

import java.time.LocalDateTime;

// 테이블 사용 기록 테이블
@Entity(name = "room_log")
@Data
public class RoomLog {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Integer id;

    @Column(nullable = false)
    // 입장 일시
    private LocalDateTime startDttm;

    @Column(nullable = false)
    private Boolean isCheckedOut;

    @Column
    private LocalDateTime endDttm;

//    @Column
//    // 사용 인원
//    private Byte persons;

    @Column(nullable = false)
    // 테마 재생 여부
    private Boolean useTheme;

    // 테이블 ID
    @ManyToOne
    @JoinColumn(name="room_id")
    private Room room;

    @ManyToOne
    @JoinColumn(name="stock_id")
    private Stock stock;

    @PrePersist
    public void prePersist() {
        this.startDttm = LocalDateTime.now();
        this.isCheckedOut = false;
        this.useTheme = true;
    }
}
