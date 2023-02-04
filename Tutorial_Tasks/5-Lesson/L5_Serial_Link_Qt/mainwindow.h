/*
   Wireless Sensor Networks Laboratory

   Technische Universität München
   Lehrstuhl für Kommunikationsnetze
   http://www.lkn.ei.tum.de

   copyright (c) 2014 Chair of Communication Networks, TUM

   contributors:
   * Thomas Szyrkowiec
   * Mikhail Vilgelm
   * Octavio Rodríguez Cervantes

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 4: Serial Link - Reading the serial port through a GUI using Qt.
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include "qextserialport.h"         // Enables use of the qextserialport library.
#include "qextserialenumerator.h"   // Helps list of open ports.

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

// ----------- METHODS -----------
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

/* In GUI programming, when a widget is changed, we often want another widget
 * to be notified.
 *
 * For instance, if a user clicks the "open" button, we probably want the
 * port's open() function to be called.
 *
 * This is traditionally achieved using callback functions, which bring
 * two  major drawbacks: callbacks are not type-safe and the calling function
 * must know which callback to call.
 *
 * To avoid these problems, Qt uses signals and slots instead:
 *  - A signal is emitted when a particular event occurs.
 *  - A slot is a function that is called in response to a particular signal.
 *
 * For more details check:
 * http://qt-project.org/doc/qt-4.8/signalsandslots.html
 *
 * For an even deeper look, check:
 * http://woboq.com/blog/how-qt-signals-slots-work.html
 */
private slots:
        void on_pushButton_close_clicked(); // Opens a port.
        void on_pushButton_open_clicked();  // Closes a port.
        void receive();                     // Receives data from a port.

        void on_pushButton_send_clicked();  // Sends the command specified.

// ----------- ATTRIBUTES -----------
private:
    Ui::MainWindow *ui;
    QextSerialPort port;            // Creates a serial port instance.
    QMessageBox error;              // USed to process error messages.
};

#endif // MAINWINDOW_H
