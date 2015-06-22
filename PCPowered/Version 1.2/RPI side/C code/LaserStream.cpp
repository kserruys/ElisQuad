#include "Communicator.h"
#include <zmq.h>

#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <mutex>
#include <thread>

using namespace std;


mutex mtx;
int dx = 0,dy = 0,yaw = 0,tyaw = 0,dt = 0, tt=0;


void recvOdometry(Communicator* comm){
    char recvBuf[30];

    while(1){
        comm->recv(recvBuf, 30, 0);

        mtx.lock();
        //get dx
        int length = 0;
        int index1 = 0;
        int index2;
        while(recvBuf[index1] != '|')
            index1++;

        index2 = ++index1;
        while(recvBuf[index2] != '|')
            index2++;

        length = index2-index1;
        char dxBuf[length];
        strncpy(dxBuf, recvBuf+index1, length);
        dx = atoi(dxBuf);

        //get dy
        index1 = ++index2;
        while(recvBuf[index2] != '|')
            index2++;

        length = index2-index1;
        char dyBuf[length];
        strncpy(dyBuf, recvBuf+index1, length);
        dy = atoi(dyBuf);

        //get dt
        index1 = ++index2;
        while(recvBuf[index2] != '|')
            index2++;

        length = index2-index1;
        char dtBuf[length];
        strncpy(dtBuf, recvBuf+index1, length);
        dt = atoi(dtBuf);

        //get yaw
        index1 = ++index2;
        while(recvBuf[index2] != '|')
            index2++;

        length = index2-index1;
        char yawBuf[length];
        strncpy(yawBuf, recvBuf+index1, length);
        yaw = atoi(yawBuf);

        tt += dt;
        tyaw += yaw;
        mtx.unlock();
    }


}

int main(int argc, char *argv[]){
	Communicator* comm = new Communicator(512, "10.42.0.1", 8000, "*", 8002);

    thread t(recvOdometry, comm);

	//initialize laser
    urg_t urg;
    long int *scan;
    long max_distance = 5000;
    long min_distance = 150;
    long time_stamp;
    int i;
    int n;

    //check if laser is available
    if (open_urg_sensor(&urg, argc, argv) < 0) {
        return -1;
    }

    //allocate necessary memory for scanning
    int numberOfPoints = 685;
    int sizeOfScan = sizeof(int);
    int bytesPerScan = numberOfPoints*sizeOfScan;
    scan = (long int*)malloc(urg_max_data_size(&urg) * sizeof(scan[0]));
    if (!scan) {
        perror("urg_max_index()");
        return -1;
    }

    //allocate memory for unsigned int
    int* intScan = (int*) malloc(bytesPerScan);


    //give information to PC about bytes per scan
    char buf[4];
    sprintf(buf, "%d", bytesPerScan);
    comm->send(buf, 4, 0);

    cout << numberOfPoints << endl;
    cout << sizeOfScan << endl;

    while(1){
    	//get new scan
        urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
        n = urg_get_distance(&urg, scan, &time_stamp);
        if (n < 0) {
            printf("urg_get_distance: %s\n", urg_error(&urg));
            urg_close(&urg);
            return -1;
        }

        //first four values are odometry
        //put latest odometry-data in scan.
        mtx.lock();
        intScan[0] = dx;
        intScan[1] = dy;
        intScan[2] = tyaw;
        intScan[3] = tt;
        tt = 0;
        tyaw = 0;
        mtx.unlock();

        //better to parse to unsigned ints --> less data will be sent
        for(int i=4; i<numberOfPoints; i++){
            intScan[i] = (int)scan[i];
        }

        //send new scan
        comm->send(intScan, bytesPerScan, 0);
    }

    t.join();

    delete scan;
    urg_close(&urg);
}
