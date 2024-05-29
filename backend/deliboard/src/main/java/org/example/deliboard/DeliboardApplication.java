package org.example.deliboard;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableAsync;

@EnableAsync
@SpringBootApplication
public class DeliboardApplication {

    public static void main(String[] args) {
        SpringApplication.run(DeliboardApplication.class, args);
    }

}
