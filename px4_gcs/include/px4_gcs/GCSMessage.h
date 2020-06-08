//
// Created by lijiayi on 2020/5/16.
//

#ifndef SRC_GCSMESSAGE_H
#define SRC_GCSMESSAGE_H

#include <vector>
#include "px4_message_type.h"
#include <serial/serial.h>

#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)    )  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )


// TYPE:
//    GCS SEND :1 COMMAND 2 CURRENT_STATE 3 SETPOINT
//    UAV SEND :4 MESSAGE_BACK

//GCS SEND
// COMMADN:
//   FRAME:    0XFE+0X22+MESSAGE_TYPE+MODE+OFFBOARD_COMMAND+ARMED_COMMAND+TAKEOFF_COMMAND+BACK_INFO+SUMCHECK+0XEE

// CURRENT_STATE:
//   FRAME:    0XFE+0X22+MESSAGE_TYPE+CURRENT_STATE+3*INT(X,Y,Z)(12*BYTE)/3*INT(R,P,y)(12*BYTE)+SUMCHECK+0XEE

// SETPOINT:
//   FRAME:    OXFE+OX22+MESSAGE_TYPE+SETPOINT_TYPE+3*INT(X,Y,Z)(12*BYTE)/4*INT(R,P,Y,T)(16*BYTE)+SUMCHECK+OXEE

// UAVSEND
// FRAME:    0XFE+0X22+MESSAGE_TYPE+BACK_INFO+3*INT(X,Y,Z)(12*BYTE)/3*INT(R,P,y)(12*BYTE)+SUMCHECK+0XEE

class GCSMessage {
public:
    MESSAGE_TYPE messageType;

    GCSMessage(MESSAGE_TYPE messageType) : messageType(messageType) {}
};

class CMDMessage : public GCSMessage {
public:
    MODE mode;
    OFFBOARD_COMMAND offboardCommand;
    ARM_COMMAND armCommand;
    TAKEOFF_COMMAND takeoffCommand;
    BACK_INFO backInfo;


    // CMDMessage() : GCSMessage(COMMAND), mode(VICON), offboardCommand(OFF_DO_NOTHING),
    //                         armCommand(ARM_DO_NOTHING),
    //                         takeoffCommand(TAKEOFF_DO_NOTHING), backInfo(DO_NOTHING) {}

    CMDMessage(MODE mode) : GCSMessage(COMMAND), mode(mode), offboardCommand(OFF_DO_NOTHING),
                            armCommand(ARM_DO_NOTHING),
                            takeoffCommand(TAKEOFF_DO_NOTHING), backInfo(DO_NOTHING) {}
};

// CURRENT_STATE:
//   FRAME:    0XFE+0X22+MESSAGE_TYPE+CURRENT_STATE+3*INT(X,Y,Z)(12*BYTE)/3*INT(R,P,y)(12*BYTE)+SUMCHECK+0XEE

class CSTMessage : public GCSMessage {
public:
    SEND_CURRENT_STATE sendCurrentState;
    std::vector<double> position;
    std::vector<double> rpy;

    CSTMessage(SEND_CURRENT_STATE sendCurrentState) : GCSMessage(CURRENT_STATE), sendCurrentState(sendCurrentState),
                                                      position(3), rpy(3) {}
};

// SETPOINT:
//   FRAME:    OXFE+OX22+MESSAGE_TYPE+SETPOINT_TYPE+3*INT(X,Y,Z)(12*BYTE)/4*INT(R,P,Y,T)(16*BYTE)+SUMCHECK+OXEE

class SPMessage : public GCSMessage {
public:
    SETPOINT_TYPE setpointType;
    std::vector<double> xyz;
    std::vector<double> rpyt;

    SPMessage(SETPOINT_TYPE setpointType) : GCSMessage(SETPOINT), setpointType(setpointType), xyz(3), rpyt(3) {}
};


class UAVMessage {
public:
    BACK_INFO backInfo;
    std::vector<float> position;
    std::vector<float> acceleration;

    UAVMessage() : position(3), acceleration(3) {}
};

class GCSMessageManager {
public:
    serial::Serial &serialport;
    bool showOutput = false;

    GCSMessageManager(serial::Serial &serialport) : serialport(serialport) {
        if (!serialport.isOpen())
            std::cout << "WARNING: serial port is NOT open!" << std::endl;
    }

    void sendMsg(CMDMessage msg) {
        sendHead();
        unsigned char sumCheck =
                msg.messageType + msg.mode + msg.offboardCommand + msg.armCommand + msg.takeoffCommand + msg.backInfo;
        unsigned char content[] = {msg.messageType, msg.mode, msg.offboardCommand, msg.armCommand, msg.takeoffCommand,
                                   msg.backInfo,
                                   sumCheck};
        if(showOutput) for(int i = 0;i<sizeof(content);i++)
            {
                printf(" %X",content[i]);
            }
        serialport.write(content, sizeof(content));
        sendTail();
        if(showOutput) printf("\n");
    }

    void sendMsg(CSTMessage msg) {
        sendHead();
        unsigned char sumCheck = 0;
//        unsigned char sumCheck = msg.messageType + msg.sendCurrentState;
        unsigned char content[] = {msg.messageType, msg.sendCurrentState};
        serialport.write(content, sizeof(content));
        if (msg.sendCurrentState == SEND_NED_POSITION_RPY || msg.sendCurrentState == SEND_NED_POSITION)
        {
            for (double i : msg.position) sumCheck += sendInteger(int(i*SCALE));
        }
        if (msg.sendCurrentState == SEND_NED_POSITION_RPY || msg.sendCurrentState == SEND_NED_RPY)
        {
            for (double i : msg.rpy) sumCheck += sendInteger(int(i*SCALE));
        }
        serialport.write(&sumCheck, 1);
        sendTail();
    }

    void sendMsg(SPMessage msg) {
        sendHead();
        unsigned char sumCheck = 0;
//        unsigned char sumCheck = msg.messageType + msg.setpointType;
        unsigned char content[] = {msg.messageType, msg.setpointType};
        serialport.write(content, sizeof(content));
        if (msg.setpointType == LOCAL_POSITION_SP || msg.setpointType == VELOCITY_SP)
        {
            for (double i : msg.xyz) sumCheck += sendInteger(int(i*SCALE));
        } else if (msg.setpointType == GLOBAL_POSITION_SP || msg.setpointType == ATTITUDE_SP)
        {
            for (double i : msg.rpyt) sumCheck += sendInteger(int(i*SCALE));
        }
        serialport.write(&sumCheck, 1);
        sendTail();
    }

    UAVMessage receiveOneMsg() {
        int status = 0;
        UAVMessage uavMessage = UAVMessage();
        bool continueLoop = true;
        unsigned char sumCheck = 0;
        while (continueLoop && serialport.available())
        {
            std::string str;
            switch (status)
            {
                case 0:
                    str = serialport.read();
                    if (str[0] == (char) 0xfe) status = 1;
                    sumCheck = 0;
                    break;
                case 1:
                    str = serialport.read();
                    if (str[0] == (char) 0x22) status = 2;
                    else status = 0;
                    break;
                case 2:
                    str = serialport.read();
                    if (str[0] == (char) MESSAGE_BACK) status = 3;
                    else
                    {
                        status = 0;
                        std::cout << "WARNING: MESSAGE_TYPE is not MESSAGE_BACK!" << std::endl;
                    }
                    //sumCheck += MESSAGE_BACK;
                    break;
                case 3:
                    str = serialport.read();
                    uavMessage.backInfo = static_cast<BACK_INFO>(str[0]);
                    if (str[0] == (char) ONLY_POSITION) status = 4;
                    else if (str[0] == (char) ONLY_IMU) status = 5;
                    else if (str[0] == (char) POSITION_AND_IMU) status = 6;
                    else
                    {
                        status = 0;
                        std::cout << "WARNING: BACK_INFO is undefined!" << std::endl;
                    }
                    //sumCheck += str[0];
                    if (status != 4 && status != 6) break;
                case 4:  //ONLY_POSITION
                    str = serialport.read(12);
                    if (str.size() < 12)
                    {
                        continueLoop = false;
                        std::cout << "WARNING: waiting for int TIMEOUT!" << std::endl;
                        break;
                    }
                    for (int i = 0; i < 3; i++)
                    {
                        int data = 0;
                        for (int j = i * 4; j < i * 4 + 4; j++)
                        {
                            sumCheck += str[j];
                            data = (data << 8)|(u_char)str[j];
                        }
                        uavMessage.position[i] = data / SCALE;
                    }
                    if (status == 4)
                    {
                        status = 7;
                        break;
                    }
                case 5:
                    str = serialport.read(12);
                    if (str.size() < 12)
                    {
                        continueLoop = false;
                        std::cout << "WARNING: waiting for int TIMEOUT!" << std::endl;
                        break;
                    }
                    for (int i = 0; i < 3; i++)
                    {
                        int data = 0;
                        for (int j = i * 4; j < i * 4 + 4; j++)
                        {
                            sumCheck += str[j];
                            data = (data << 8) | ((unsigned char)str[j]);
                        }
                        uavMessage.acceleration[i] = data / SCALE;
                    }
                    status = 7;
                    break;
                case 6:
                    status = 0;
                    break;
                case 7:
                    str = serialport.read();
                    if (str[0] == (char)sumCheck) status = 8;
                    else
                    {
                        status = 0;
                        std::cout << "SumCheck ERROR: from serial: ";
                        printf("%X",str[0]);
                        std::cout << " by calculation: ";
                        printf("%X",sumCheck);
                        std::cout << std::endl;
                    }
                    break;
                case 8:
                    str = serialport.read();
                    if (str[0] == (char) 0xee) continueLoop = false;
                    else
                    {
                        continueLoop = false;
                        std::cout << "WARNING: final byte 0xee Lost!" << std::endl;
                    }
                    break;
            }
        }
        return uavMessage;
    }

private:
    void sendHead() {
        unsigned char head[] = {0xfe, 0x22};
        serialport.write(head, 2);
    }

    void sendTail() {
        unsigned char tail[] = {0xee};
        serialport.write(tail, 1);
    }

    unsigned char sendInteger(int intToSend) {
        serialport.write((uint8_t *) (&intToSend), sizeof(int));
        return BYTE0(intToSend) + BYTE1(intToSend) + BYTE2(intToSend) + BYTE3(intToSend);
    }
};

#endif //SRC_GCSMESSAGE_H
