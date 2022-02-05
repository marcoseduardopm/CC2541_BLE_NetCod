
#ifndef MYDATA_H_
#define MYDATA_H_


struct tagMyData{
    char address[10][6];
    char payload[10][3];
    char line;
};
struct tagReceiveData{
    char address[6];
    char payload[3];
    char line;
};
struct tagMyDevice{
    char id;
    char address[6];
    char payload[3];
    
    union{
        char  flags;
        struct{
            char msgSend:1;
            char msgReceived:1;
            char msgCombined:1;
            char msgCombinedSend:1;
        };
    };
    int timer;
};

extern struct tagMyData myDatasA;
extern struct tagMyData myDatasB;
extern struct tagMyDevice myDevice;
extern struct tagReceiveData receiveData;


#endif /* MYDATA_H_ */