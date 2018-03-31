#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import subprocess
from threading import Thread
import time

class Display(QWidget):
    def __init__(self):
        super(Display, self).__init__()

        self.setGeometry(0, 0, 1460, 130)
        self.setWindowTitle('Icon')
        self.setWindowIcon(QIcon('web.png'))

        QToolTip.setFont(QFont('SansSerif', 10))

        self.setBackgroundRole(QPalette.NoRole)

        # btn = QPushButton('Quit', self)
        # btn.setToolTip('This is a <b>QPushButton</b> widget')
        # btn.resize(btn.sizeHint())
        # btn.clicked.connect(QCoreApplication.instance().quit)
        # btn.move(50, 50)

        # b1 = QPushButton('Reder', self)
        # b1.resize(b1.sizeHint())
        # b1.clicked.connect(self.reder_clicked)
        # b1.move(50, 100)
        self.rgb_value = (126, 126, 126)

        self.lat = QLabel('37.70724976', self)
        self.lat.setFont(QFont('SansSerif', 50, QFont.Bold))
        self.lat.move(100, 20)
        self.lat.adjustSize()

        self.lon = QLabel('-121.42293867', self)
        self.lon.setFont(QFont('SansSerif', 50, QFont.Bold))
        self.lon.move(690, 20)
        self.lon.adjustSize()

        self.ns = QLabel('00', self)
        self.ns.setFont(QFont('SansSerif', 50, QFont.Bold))
        self.ns.move(1350, 20)
        self.ns.adjustSize()

    def reder_clicked(self, event):
        self.red_value += 20
        self.repaint()

    def paintEvent(self, event):
        paint = QPainter()
        paint.begin(self)
        paint.setRenderHint(QPainter.Antialiasing)
        radx = 25
        rady = 25
        paint.setPen(Qt.black)
        # print "mw %d %d %d" % self.rgb_value

        paint.setBrush(QColor(self.rgb_value[0], self.rgb_value[1], self.rgb_value[2]))
        center = QPoint(50, 60)
        paint.drawEllipse(center, radx, rady)
        paint.end()

class Reader(Thread):
    def __init__(self, group = None, target = None, name = None,
                 args = (), kwargs = None, verbose = None):
        Thread.__init__(self, group = group, target = target, name = name, verbose = verbose)
        self.args = args
        self.kwargs = kwargs
        return

    def run(self):
        d = self.args[0]
        fn = subprocess.check_output(["/export/home/marcow/bin/find_last", "sol1_*.pos"]).rstrip()

        fin = open(fn, 'r')
        last_update = 0

        while True:
            line = fin.readline()

            if not line:
                where = fin.tell()
                time.sleep(0.2)
                fin.seek(where)
            else:
                elems = line.rstrip().split()
                i = 0
                try:
                    i = int(elems[0])
                except ValueError:
                    i = 0

                # print "mw i= %d  line= %s" % (i, line.rstrip())

                now = time.time()
                # print "mw now= %f  %f" % (last_update, now)
                if (i > 1000) and (now - last_update > 0.1):
                    last_update = now
                    # print "mw update"
                    # valid lines
                    d.lat.setText("%.8f" % float(elems[2]))
                    d.lat.adjustSize()
                    d.lon.setText("%.8f" % float(elems[3]))
                    d.lon.adjustSize()
                    d.ns.setText("%2d" % int(elems[6]))
                    d.ns.adjustSize()

                    q = int(elems[5])

                    if q == 1: # tourquoise
                        d.rgb_value = (50, 145, 174)
                    elif q == 2:
                        d.rgb_value = (0, 200, 0)
                    elif q == 3 or q == 4:
                        d.rgb_value = (220, 200, 33)
                    elif q == 5:
                        d.rgb_value = (236, 168, 21)
                    else:
                        d.rgb_value = (255, 0, 0)

                    d.update()

        return


def main():
    app = QApplication(sys.argv)
    d = Display()
    d.show()
    r = Reader(args = (d,))
    r.daemon = True
    r.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

