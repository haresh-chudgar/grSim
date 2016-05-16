#include "mainwindow.h"
#include <QGridLayout>
#include <QDebug>
#include <QMessageBox>

#define NUM_ROBOTS 6

// Display Settings for the control GUI
MainWindow::MainWindow(QWidget *parent) : QDialog(parent), udpsocket(this), communicator(&udpsocket), bluebook(PlayBook::TheBlueBook()), ybook(PlayBook::TheYellowBook()),
      blueTeam(false, &communicator, &bluebook, NUM_ROBOTS), yellowTeam(true, &communicator, &ybook, NUM_ROBOTS)
{
  fieldInfoSocket = NULL;
  listenToGRSim();
  
  SoccerFieldInfo::CreateInstance(&blueTeam, &yellowTeam);
  
  QGridLayout* layout = new QGridLayout(this);
  edtIp = new QLineEdit("127.0.0.1", this);
  edtPort = new QLineEdit("20011", this);
  edtId = new QLineEdit("0", this);
  edtVx = new QLineEdit("0", this);
  edtVy = new QLineEdit("0", this);
  edtW  = new QLineEdit("0", this);
  edtV1 = new QLineEdit("0", this);
  edtV2 = new QLineEdit("0", this);
  edtV3 = new QLineEdit("0", this);
  edtV4 = new QLineEdit("0", this);
  edtChip = new QLineEdit("0", this);
  edtKick = new QLineEdit("0", this);

  this->setWindowTitle(QString("grSim Sample Client - v 1.0"));

  lblIp = new QLabel("Simulator Address", this);
  lblPort = new QLabel("Simulator Port", this);

  lblId = new QLabel("Id", this);
  lblVx = new QLabel("Velocity X (m/s)", this);
  lblVy = new QLabel("Velocity Y (m/s)", this);
  lblW  = new QLabel("Velocity W (rad/s)", this);
  lblV1 = new QLabel("Wheel1 (rad/s)", this);
  lblV2 = new QLabel("Wheel2 (rad/s)", this);
  lblV3 = new QLabel("Wheel3 (rad/s)", this);
  lblV4 = new QLabel("Wheel4 (rad/s)", this);
  cmbTeam = new QComboBox(this);
  cmbTeam->addItem("Yellow");
  cmbTeam->addItem("Blue");
  lblChip = new QLabel("Chip (m/s)", this);
  lblKick = new QLabel("Kick (m/s)", this);
  txtInfo = new QTextEdit(this);
  chkVel = new QCheckBox("Send Velocity? (or wheels)", this);
  chkSpin = new QCheckBox("Spin", this);
  btnSend = new QPushButton("Send", this);
  btnReset = new QPushButton("Reset", this);
  btnConnect = new QPushButton("Connect", this);
  txtInfo->setReadOnly(true);
  txtInfo->setHtml("This program is part of <b>grSim RoboCup SSL Simulator</b> package.<br />For more information please refer to <a href=\"http://eew.aut.ac.ir/~parsian/grsim/\">http://eew.aut.ac.ir/~parsian/grsim</a><br /><font color=\"gray\">This program is free software under the terms of GNU General Public License Version 3.</font>");
  txtInfo->setFixedHeight(70);
  layout->addWidget(lblIp, 1, 1, 1, 1);layout->addWidget(edtIp, 1, 2, 1, 1);
  layout->addWidget(lblPort, 1, 3, 1, 1);layout->addWidget(edtPort, 1, 4, 1, 1);
  layout->addWidget(lblId, 2, 1, 1, 1);layout->addWidget(edtId, 2, 2, 1, 1);layout->addWidget(cmbTeam, 2, 3, 1, 2);
  layout->addWidget(lblVx, 3, 1, 1, 1);layout->addWidget(edtVx, 3, 2, 1, 1);
  layout->addWidget(lblVy, 4, 1, 1, 1);layout->addWidget(edtVy, 4, 2, 1, 1);
  layout->addWidget(lblW, 5, 1, 1, 1);layout->addWidget(edtW, 5, 2, 1, 1);
  layout->addWidget(chkVel, 6, 1, 1, 1);layout->addWidget(edtKick, 6, 2, 1, 1);
  layout->addWidget(lblV1, 3, 3, 1, 1);layout->addWidget(edtV1, 3, 4, 1, 1);
  layout->addWidget(lblV2, 4, 3, 1, 1);layout->addWidget(edtV2, 4, 4, 1, 1);
  layout->addWidget(lblV3, 5, 3, 1, 1);layout->addWidget(edtV3, 5, 4, 1, 1);
  layout->addWidget(lblV4, 6, 3, 1, 1);layout->addWidget(edtV4, 6, 4, 1, 1);
  layout->addWidget(lblChip, 7, 1, 1, 1);layout->addWidget(edtChip, 7, 2, 1, 1);
  layout->addWidget(lblKick, 7, 3, 1, 1);layout->addWidget(edtKick, 7, 4, 1, 1);
  layout->addWidget(chkSpin, 8, 1, 1, 4);
  layout->addWidget(btnConnect, 9, 1, 1, 2);layout->addWidget(btnSend, 9, 3, 1, 1);layout->addWidget(btnReset, 9, 4, 1, 1);
  layout->addWidget(txtInfo, 10, 1, 1, 4);
  timer = new QTimer (this);
  timer->setInterval(20);
  connect(edtIp, SIGNAL(textChanged(QString)), this, SLOT(disconnectUdp()));
  connect(edtPort, SIGNAL(textChanged(QString)), this, SLOT(disconnectUdp()));
  connect(timer, SIGNAL(timeout()), this, SLOT(sendPacket()));
  connect(btnConnect, SIGNAL(clicked()), this, SLOT(reconnectUdp()));
  connect(btnSend, SIGNAL(clicked()), this, SLOT(sendBtnClicked()));
  connect(btnReset, SIGNAL(clicked()), this, SLOT(resetBtnClicked()));
  btnSend->setDisabled(true);
  chkVel->setChecked(true);
  sending = false;
  reseting = false;
}

MainWindow::~MainWindow()
{
  if(fieldInfoSocket != NULL && fieldInfoSocket->isValid() && fieldInfoSocket->isOpen())
    fieldInfoSocket->close();
}

// Set up socket connection with GRSIM
void MainWindow::listenToGRSim() {
  
  quint16 port = 10020;
  QHostAddress *net_address = new QHostAddress(QString("224.5.23.2"));
  QNetworkInterface *net_interface = new QNetworkInterface(QNetworkInterface::interfaceFromName(""));
  
  if (fieldInfoSocket!=NULL)
  {
      QObject::disconnect(fieldInfoSocket,SIGNAL(readyRead()),this,SLOT(recvFieldInfo()));
      delete fieldInfoSocket;
  }
  
  fieldInfoSocket = new QUdpSocket(this);
  bool isSuccess = fieldInfoSocket->bind(*net_address, port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
  if (isSuccess == false) {
    fprintf(stderr, "Unable to bind UDP socket on port %d: %s\n", port, fieldInfoSocket->errorString().toStdString().c_str());
  }
  if(!fieldInfoSocket->joinMulticastGroup(*net_address, *net_interface)) {
    fprintf(stderr, "Unable to join UDP multicast on %s: %d %s\n", net_address->toString().toStdString().c_str(), port, fieldInfoSocket->errorString().toStdString().c_str());
  }
  
  isSuccess = QObject::connect(fieldInfoSocket,SIGNAL(readyRead()),this,SLOT(recvFieldInfo()));
}

// Fills SoccerFieldInfo class
void MainWindow::recvFieldInfo()
{
  QHostAddress sender;
  quint16 port;
  int size = 0;
  char in_buffer[65536];
  while (fieldInfoSocket->hasPendingDatagrams())
  {
    size = fieldInfoSocket->readDatagram(in_buffer, 65536, &sender, &port);
  }
  if(size == 0)
    return;
  
  SoccerFieldInfo::Instance()->receive(in_buffer, size);
}

void MainWindow::disconnectUdp()
{
    udpsocket.close();
    sending = false;
    btnSend->setText("Send");
    btnSend->setDisabled(true);
}

void MainWindow::sendBtnClicked()
{
    sending = !sending;
    if (!sending)
    {
        timer->stop();
        btnSend->setText("Send");
    }
    else {
        timer->start();
        btnSend->setText("Pause");
    }
}

void MainWindow::resetBtnClicked()
{
    reseting = true;
    edtVx->setText("0");
    edtVy->setText("0");
    edtW->setText("0");
    edtV1->setText("0");
    edtV2->setText("0");
    edtV3->setText("0");
    edtV4->setText("0");
    edtChip->setText("0");
    edtKick->setText("0");
    chkVel->setChecked(true);
    chkSpin->setChecked(false);
}


void MainWindow::reconnectUdp()
{
  _addr = edtIp->text();
  _port = edtPort->text().toUShort();
  btnSend->setDisabled(false);
  communicator.reconnectUdp(_addr, _port);
}

void MainWindow::sendPacket()
{
    if (reseting)
    {
        sendBtnClicked();
        reseting = false;
    }
    grSim_Packet packet;
    bool yellow = false;
    if (cmbTeam->currentText()=="Yellow") yellow = true;
    packet.mutable_commands()->set_isteamyellow(yellow);
    packet.mutable_commands()->set_timestamp(0.0);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(edtId->text().toInt());

    command->set_wheelsspeed(!chkVel->isChecked());
    command->set_wheel1(edtV1->text().toDouble());
    command->set_wheel2(edtV2->text().toDouble());
    command->set_wheel3(edtV3->text().toDouble());
    command->set_wheel4(edtV4->text().toDouble());
    command->set_veltangent(edtVx->text().toDouble());
    command->set_velnormal(edtVy->text().toDouble());
    command->set_velangular(edtW->text().toDouble());

    command->set_kickspeedx(edtKick->text().toDouble());
    command->set_kickspeedz(edtChip->text().toDouble());
    command->set_spinner(chkSpin->isChecked());

    QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    udpsocket.writeDatagram(dgram, _addr, _port);
}
