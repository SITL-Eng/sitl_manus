#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Transform.h"
#include "sitl_manus/glove.h"

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")


#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

geometry_msgs::Transform set_msg(geometry_msgs::Transform msg, std::vector<float> receivedFloatVector, int begin){
    msg.translation.x = receivedFloatVector[begin];
    msg.translation.y = receivedFloatVector[begin+1];
    msg.translation.z = receivedFloatVector[begin+2];
    msg.rotation.x    = receivedFloatVector[begin+3];
    msg.rotation.y    = receivedFloatVector[begin+4];
    msg.rotation.z    = receivedFloatVector[begin+5];
    msg.rotation.w    = receivedFloatVector[begin+6];
    return msg;
}


int __cdecl main(int argc, char **argv) 
{
    ros::init(argc, argv, "pub_manus_glove");
    ros::NodeHandle n;
    ros::Publisher tf_pub = n.advertise<geometry_msgs::Transform>("/manus/tf", 1000);
    ros::Publisher glove_pub  = n.advertise<sitl_manus::glove>("/manus/cp", 1000);
    // ros::Publisher glove_node2_pub  = n.advertise<geometry_msgs::TransformStamped>("/manus/node2/cp", 1000);
    // ros::Publisher glove_node3_pub  = n.advertise<geometry_msgs::TransformStamped>("/manus/node3/cp", 1000);
    // ros::Publisher glove_node4_pub  = n.advertise<geometry_msgs::TransformStamped>("/manus/node4/cp", 1000);
    // ros::Publisher glove_node5_pub  = n.advertise<geometry_msgs::TransformStamped>("/manus/node5/cp", 1000);
    // ros::Publisher glove_node6_pub  = n.advertise<geometry_msgs::TransformStamped>("/manus/node6/cp", 1000);
    // ros::Publisher glove_node7_pub  = n.advertise<geometry_msgs::TransformStamped>("/manus/node7/cp", 1000);
    // ros::Publisher glove_node8_pub  = n.advertise<geometry_msgs::TransformStamped>("/manus/node8/cp", 1000);
    // ros::Publisher glove_node9_pub  = n.advertise<geometry_msgs::TransformStamped>("/manus/node9/cp", 1000);
    // ros::Publisher glove_node10_pub = n.advertise<geometry_msgs::TransformStamped>("/manus/node10/cp", 1000);
    // ros::Publisher glove_node11_pub = n.advertise<geometry_msgs::TransformStamped>("/manus/node11/cp", 1000);
    // ros::Publisher glove_node12_pub = n.advertise<geometry_msgs::TransformStamped>("/manus/node12/cp", 1000);
    // ros::Publisher glove_node13_pub = n.advertise<geometry_msgs::TransformStamped>("/manus/node13/cp", 1000);
    // ros::Publisher glove_node14_pub = n.advertise<geometry_msgs::TransformStamped>("/manus/node14/cp", 1000);
    // ros::Publisher glove_node15_pub = n.advertise<geometry_msgs::TransformStamped>("/manus/node15/cp", 1000);
    // ros::Publisher glove_node16_pub = n.advertise<geometry_msgs::TransformStamped>("/manus/node16/cp", 1000);
    // ros::Publisher glove_node17_pub = n.advertise<geometry_msgs::TransformStamped>("/manus/node17/cp", 1000);

    ros::Rate loop_rate(70);

    WSADATA wsaData;
    SOCKET ConnectSocket = INVALID_SOCKET;
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;
    const char *sendbuf = "this is a test";
    char recvbuf[DEFAULT_BUFLEN];
    int iResult;
    int recvbuflen = DEFAULT_BUFLEN;
    float receivedRotX = 0.0f; // Local variable to store the received rotx value

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Attempt to connect to an address until one succeeds
    for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

        // Create a SOCKET for connecting to server
        ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
            ptr->ai_protocol);
        if (ConnectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return 1;
        }

        // Connect to server.
        iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(ConnectSocket);
            ConnectSocket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (ConnectSocket == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return 1;
    }


    // Allocate memory for the vector data
    std::vector<float> receivedFloatVector(119);
    size_t vectorSize=119;
    geometry_msgs::Transform tf;
    sitl_manus::glove msg;

    // Receive until the peer closes the connection
    do {
        // Receive the size of the vector first
        
        //int iRecvResult = recv(ConnectSocket, (char*)&vectorSize, sizeof(size_t), 0);

        // Check for errors in iRecvResult here...

        // Receive the vector data
        iResult = recv(ConnectSocket, (char*)receivedFloatVector.data(), sizeof(float) * vectorSize, 0);

        //receive rotx
        //iResult = recv(ConnectSocket, (char*)&receivedRotX, sizeof(float), 0);
        
        if ( iResult > 0 ){           
            // printf("Received Rotx %f\n", receivedRotX);
            // Print the received data
            // publishing(ros::Publisher pub, geometry_msgs::TransformStamped msg, int begin)
            // tf = set_msg(tf,receivedFloatVector,0);
            tf.translation.x = receivedFloatVector[0];
            tf.translation.y = receivedFloatVector[0+1];
            tf.translation.z = receivedFloatVector[0+2];
            tf.rotation.x    = receivedFloatVector[0+3];
            tf.rotation.y    = receivedFloatVector[0+4];
            tf.rotation.z    = receivedFloatVector[0+5];
            tf.rotation.w    = receivedFloatVector[0+6];
            msg.transform1 = tf;
            // tf = set_msg(tf,receivedFloatVector,7);
            // msg.transform2 = tf;
            // tf = set_msg(tf,receivedFloatVector,14);
            // msg.transform3 = tf;
            // tf = set_msg(tf,receivedFloatVector,21);
            // msg.transform4 = tf;
            // tf = set_msg(tf,receivedFloatVector,28);
            // msg.transform5 = tf;
            // tf = set_msg(tf,receivedFloatVector,35);
            // msg.transform6 = tf;
            // tf = set_msg(tf,receivedFloatVector,42);
            // msg.transform7 = tf;
            // tf = set_msg(tf,receivedFloatVector,49);
            // msg.transform8 = tf;
            // tf = set_msg(tf,receivedFloatVector,56);
            // msg.transform9 = tf;
            // tf = set_msg(tf,receivedFloatVector,63);
            // msg.transform10 = tf;
            // tf = set_msg(tf,receivedFloatVector,70);
            // msg.transform11 = tf;
            // tf = set_msg(tf,receivedFloatVector,77);
            // msg.transform12 = tf;
            // tf = set_msg(tf,receivedFloatVector,84);
            // msg.transform13 = tf;
            // tf = set_msg(tf,receivedFloatVector,91);
            // msg.transform14 = tf;
            // tf = set_msg(tf,receivedFloatVector,98);
            // msg.transform15 = tf;
            // tf = set_msg(tf,receivedFloatVector,105);
            // msg.transform16 = tf;
            // tf = set_msg(tf,receivedFloatVector,112);
            // msg.transform17 = tf;

            tf_pub.publish(tf);
            glove_pub.publish(msg);

            printf("Received Float Vector:\n");
            for (size_t i = 0; i < receivedFloatVector.size(); i++) {
                printf("%f\n", receivedFloatVector[i]);
            }
        }
        else if ( iResult == 0 )
            printf("Connection closed\n");
        else
            printf("recv failed with error: %d\n", WSAGetLastError());

        ros::spinOnce();
        loop_rate.sleep();
    } while(ros::ok());

    // cleanup
    closesocket(ConnectSocket);
    WSACleanup();

    return 0;
}