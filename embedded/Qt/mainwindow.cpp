#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	player = new QMediaPlayer(this);
	videoWidget = new QVideoWidget();
	playlist = new QMediaPlaylist(player);
	setCentralWidget(videoWidget);
	player->setVideoOutput(videoWidget);
	
	appDir = QCoreApplication::applicationDirPath();
	videoList = { "default","west","space","ocean","jungle","middle","horror","puzzle" };	//비디오리스트 
	nowStatus = "default";
	changeVideo(appDir + "/../src/default.mp4");   
	
	connect(this, &MainWindow::requestVideoChange, this, &MainWindow::changeVideo);		// 스레드에서 소켓 통신 수행
	thread_socket = std::thread(&MainWindow::socketThread, this);
	showFullScreen();
}

MainWindow::~MainWindow()
{
	// 스레드 종료
	thread_socket.join();
	delete ui;
}

void MainWindow::changeVideo(const QString& videoPath) {	//영상 바꾸기
	playlist->clear();
	playlist->addMedia(QUrl::fromLocalFile(videoPath));
	playlist->setCurrentIndex(0);
	playlist->setPlaybackMode(QMediaPlaylist::Loop);

	player->setPlaylist(playlist);
	player->play();
}
void MainWindow::socketThread() {
	QTcpSocket socket;
	while (true) {
		if (socket.state() != QAbstractSocket::ConnectedState) {
			socket.connectToHost("j10a210.p.ssafy.io", 54321); // 서버에 연결
			if (socket.waitForConnected()) {
				std::cout << "Connected to server." << std::endl;
				QString m = "pi";
				socket.write(m.toUtf8());
				socket.flush();
			}
			else {
				std::cerr << "Failed to connect to server. Retrying ..." << std::endl;
				QThread::msleep(1000);
				continue;
			}
		}

		if (socket.waitForReadyRead()) {
			QString requestData = socket.readAll();
			std::cout << "Received request: " << requestData.toStdString() << std::endl;

			if (requestData == nowStatus) {		//이미 같은 테마
				std::cout << "Already Playing" <<std::endl;
				continue;
			}
			if (std::find(videoList.begin(), videoList.end(), requestData) != videoList.end()) {	// 요청에 따라 영상 변경
				nowStatus = requestData;
				emit requestVideoChange(appDir + "/../src/" + requestData + ".mp4");
				std::cout << "Change Complete!" << std::endl;
			}
			else
				std::cerr << "Invalid request: " << requestData.toStdString() << std::endl;
		}

		if (socket.state() == QAbstractSocket::UnconnectedState) {
			std::cerr << "Connection to server lost. Retrying second..." << std::endl;
			QThread::msleep(1000);
		}
	}
}
