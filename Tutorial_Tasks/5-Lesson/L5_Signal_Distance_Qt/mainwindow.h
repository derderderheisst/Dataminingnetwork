#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QClipboard>
#include "uart.h"

#define SERIAL_PACKET_TYPE_CONFIGURE_TEST   0
#define SERIAL_PACKET_TYPE_POWER_TEST       1

namespace Ui {
    class MainWindow;
}

typedef struct {
    unsigned char number;
    signed char tx_power;
    signed char rssi;
} RadioTest;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MainWindow *ui;
    QMessageBox error;
    bool m_record;
    Uart *uart;

private slots:


private slots:
    void on_pushButton_close_clicked();
    void on_pushButton_open_clicked();
    void receive(QString str);
    void send(QByteArray data);
    void on_pushButton_start_clicked();
    void on_pushButton_stop_clicked();
    void packet_received(QByteArray str);
    void on_pushButton_copyTable_clicked();
    void on_pushButton_clearTable_clicked();
    void on_pushButtonSetPower_clicked();
};

#endif // MAINWINDOW_H
