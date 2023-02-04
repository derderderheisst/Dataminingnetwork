#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    m_record = false;
    ui->setupUi(this);
    // Get available COM Ports
    this->uart = new Uart(this);
    QList<QextPortInfo> ports = uart->getUSBPorts();
    for (int i = 0; i < ports.size(); i++) {
        ui->comboBox_Interface->addItem(ports.at(i).portName.toLocal8Bit().constData());
    }
    QObject::connect(uart, SIGNAL(debugReceived(QString)), this, SLOT(receive(QString)));
    QObject::connect(uart, SIGNAL(packetReceived(QByteArray)), this, SLOT(packet_received(QByteArray)));
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::changeEvent(QEvent *e) {
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::on_pushButton_open_clicked() {
    QString portname = "/dev/";
    portname.append(ui->comboBox_Interface->currentText());
    uart->open(portname);
    if (!uart->isOpen())
    {
        error.setText("Unable to open port!");
        error.show();
        return;
    }


    ui->pushButton_close->setEnabled(true);
    ui->pushButton_open->setEnabled(false);
    ui->comboBox_Interface->setEnabled(false);
}

void MainWindow::on_pushButton_close_clicked() {
    if (uart->isOpen()) uart->close();
    ui->pushButton_close->setEnabled(false);
    ui->pushButton_open->setEnabled(true);
    ui->comboBox_Interface->setEnabled(true);
}

void MainWindow::receive(QString str) {
    ui->textEdit_Status->append(str);
    ui->textEdit_Status->ensureCursorVisible();
}

void MainWindow::send(QByteArray data) {
    uart->send(data);
}

void MainWindow::packet_received(QByteArray str) {
    if (str.length() == 0) return;

    switch (str.at(0)) {
    case SERIAL_PACKET_TYPE_POWER_TEST:
        RadioTest radioTest;
        radioTest.number = str.at(1);
        radioTest.tx_power = str.at(2);
        radioTest.rssi = str.at(3);

        int row = ui->tableWidget->rowCount();
        row++;
        ui->tableWidget->setRowCount(row);
        ui->tableWidget->scrollToBottom();
        QTableWidgetItem *index = new QTableWidgetItem(QString::number(radioTest.number));
        ui->tableWidget->setItem(row-1, 0, index);
        QTableWidgetItem *distance = new QTableWidgetItem(QString::number(ui->doubleSpinBox_distance->value())); //tr("%1 %2").arg(row).arg(str)
        ui->tableWidget->setItem(row-1, 1, distance);
        QTableWidgetItem *TxPower = new QTableWidgetItem(QString::number(radioTest.tx_power));
        ui->tableWidget->setItem(row-1, 2, TxPower);
        QTableWidgetItem *RxPower = new QTableWidgetItem(QString::number(radioTest.rssi));
        ui->tableWidget->setItem(row-1, 3, RxPower);

        break;
    }
}

void MainWindow::on_pushButton_start_clicked() {
    if (!m_record) {
        ui->pushButton_start->setEnabled(false);
        ui->doubleSpinBox_distance->setEnabled(false);
        ui->pushButton_stop->setEnabled(true);
        m_record = true;
    }
}

void MainWindow::on_pushButton_stop_clicked() {
    if (m_record) {
        ui->pushButton_stop->setEnabled(false);
        ui->pushButton_start->setEnabled(true);
        ui->doubleSpinBox_distance->setEnabled(true);
        m_record = false;
    }
}

void MainWindow::on_pushButton_copyTable_clicked() {
    int rows = ui->tableWidget->rowCount();
    int cols = ui->tableWidget->columnCount();
    QString selected_text;

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            if (ui->tableWidget->item(row, col)) {
                selected_text.append(ui->tableWidget->item(row, col)->text());
                //qDebug() << row << ", " << col << ": " << ui->tableWidget->item(row, col)->text();
            }
            selected_text.append('\t');
        }
        selected_text.append('\n');
    }

    QApplication::clipboard()->setText(selected_text);
}

void MainWindow::on_pushButton_clearTable_clicked() {
    ui->tableWidget->setRowCount(0);
}

void MainWindow::on_pushButtonSetPower_clicked()
{
    QByteArray data = QByteArray((int) 2, (char) 0);
    data[0] = SERIAL_PACKET_TYPE_CONFIGURE_TEST;
    data[1] = (signed char) ui->spinBoxPower->value();
    this->send(data);
}
