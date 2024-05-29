#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <thread>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QHBoxLayout>
#include <QMediaPlaylist>
#include <QTcpSocket>
#include <QHostAddress>
#include <vector>
#include <QThread>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget* parent = nullptr);
	~MainWindow();

	QString videoFilePath;
	QMediaPlayer* player;
	QVideoWidget* videoWidget;
	QMediaPlaylist* playlist;
	std::thread thread_socket;
	std::vector<QString> videoList;
	QString nowStatus;
	QString appDir;
private:
	Ui::MainWindow* ui;

	void changeVideo(const QString& videoPath);
	void socketThread();
signals:
	void requestVideoChange(const QString& videoPath);

};
#endif // MAINWINDOW_H
