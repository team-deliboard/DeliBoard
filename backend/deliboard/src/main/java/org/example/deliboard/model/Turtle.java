package org.example.deliboard.model;

import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;

@Entity(name = "turtle")
@Data
@AllArgsConstructor
@NoArgsConstructor
public class Turtle {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(nullable = false)
    private Integer id;

    @Column(nullable = false)
    private Boolean isWorking;

    @Column(nullable = false)
    private LocalDateTime createdDttm;// 등록 일시

    @Column(nullable = false)
    private Boolean isActive;

    @Column
    private LocalDateTime dumpedDttm;

    @ManyToOne
    @JoinColumn(name="store_id")
    private Store store;

}
