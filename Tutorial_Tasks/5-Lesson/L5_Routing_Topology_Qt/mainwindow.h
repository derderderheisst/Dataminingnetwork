#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QtGui>
#include <QtCore>
#include "qextserialport.h"
#include "qextserialenumerator.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void changeEvent(QEvent *e);
    void paintEvent(QPaintEvent *e);        // For the QPainter

private:
    Ui::MainWindow *ui;
    QextSerialPort port;
    QMessageBox error;

private slots:


private slots:
    void on_pushButton_close_clicked();
    void on_pushButton_open_clicked();
    void receive();
};

#endif // MAINWINDOW_H
