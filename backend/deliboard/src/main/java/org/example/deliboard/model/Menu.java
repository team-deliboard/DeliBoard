package org.example.deliboard.model;

import jakarta.persistence.*;
//import jakarta.persistence.criteria.CriteriaBuilder;
import lombok.Data;

// 식음료 테이블
@Entity(name = "menu")
@Data
public class Menu {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(nullable = false)
    private Integer id;

    @Column(nullable = false)
    // 메뉴명
    private String name;

    @Column
    // 메뉴 타입
    private Integer type;

    @Column
    private Integer price;

    @Column
    private Boolean isAvailable;

    // 매장 ID
    @ManyToOne
    @JoinColumn(name="store_id")
    private Store store;

}

