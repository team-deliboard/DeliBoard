package org.example.deliboard.service;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.GeneralResponseDto;
import org.example.deliboard.dto.SocketGameOrderDto;
import org.example.deliboard.dto.SocketThemeDto;
import org.example.deliboard.model.Room;
import org.example.deliboard.model.RoomLog;
import org.example.deliboard.model.Stock;
import org.example.deliboard.model.Store;
import org.example.deliboard.repository.RoomRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.transaction.annotation.Transactional;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;

@Service
@Transactional
@RequiredArgsConstructor
public class SocketClientService {
    private final String HOST = System.getenv("SOCKET_SERVER_HOST");
    private final Integer PORT = Integer.parseInt(System.getenv("SOCKET_PORT"));
    private final RoomRepository roomRepository;

    public String sendGameOrderToSocket(Integer storeId, Byte roomNumber, Stock stock){
        String locationX = stock.getLocationX();
        String locationY = stock.getLocationY();
        ObjectMapper objectMapper = new ObjectMapper();
        try (Socket socket = new Socket(HOST, PORT)) {
            // Render order (SocketGameOrderDto)
            String order = objectMapper.writeValueAsString(new SocketGameOrderDto(0, storeId, roomNumber, locationX, locationY));
            // Send order
            PrintWriter request = new PrintWriter(socket.getOutputStream(), true);
            request.printf("server");
            Thread.sleep(2000);
            request.printf(order);
            // Receive response
            BufferedReader input = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            return input.readLine();
        } catch (Exception e) {
            return "error";
        }
    }
    public String sendGameReturnToSocket(RoomLog roomLog){
        Stock stock = roomLog.getStock();
        if (stock == null)
            return "error";
        String locationX = stock.getLocationX();
        String locationY = stock.getLocationY();

        ObjectMapper objectMapper = new ObjectMapper();
        try (Socket socket = new Socket(HOST, PORT)) {
            // Render order (SocketGameOrderDto)
            String order = objectMapper.writeValueAsString(
                    new SocketGameOrderDto(
                            1,
                            roomLog.getRoom().getStore().getId(),
                            roomLog.getRoom().getNumber(),
                            locationX,
                            locationY
                    )
            );
            // Send order
            PrintWriter request = new PrintWriter(socket.getOutputStream(), true);
            request.printf("server");
            Thread.sleep(2000);
            request.printf(order);
            // Receive response
            BufferedReader input = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            return input.readLine();
        } catch (Exception e) {
            return "error";
        }
    }

    public void setRoomTheme(RoomLog roomLog, Boolean useTheme){
        ObjectMapper objectMapper = new ObjectMapper();
        String theme = "default";
        if (useTheme)
            theme = roomLog.getStock().getGame().getTheme();
        try (Socket socket = new Socket(HOST, PORT)) {
            // Render order (SocketThemeDto)
            String order = objectMapper.writeValueAsString(
                    new SocketThemeDto(
                            3,
                            roomLog.getRoom().getStore().getId(),
                            roomLog.getRoom().getNumber(),
                            theme
                    )
            );
            // Send order
            PrintWriter request = new PrintWriter(socket.getOutputStream(), true);
            request.printf("server");
            Thread.sleep(2000);
            request.printf(order);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public GeneralResponseDto callTurtle(RoomLog roomLog){
        ObjectMapper objectMapper = new ObjectMapper();
        try (Socket socket = new Socket(HOST, PORT)) {
            String order = objectMapper.writeValueAsString(
                    new SocketGameOrderDto(
                            2, // Menu order
                            roomLog.getRoom().getStore().getId(),
                            roomLog.getRoom().getNumber(),
                            "-4.296541213989258",
                            "8.960509300231934"
                    )
            );
            PrintWriter request = new PrintWriter(socket.getOutputStream(), true);
            request.printf("server");
            Thread.sleep(2000);
            request.printf(order);
            Thread.sleep(2000);
            BufferedReader input = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            String response = input.readLine();
            return new GeneralResponseDto(true, "Turtle called successfully");
        } catch (Exception e) {
            return new GeneralResponseDto(false, "Error while calling turtle");
        }
    }
}
