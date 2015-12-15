/*
 * Author: Evan Beachly
 * Date: August 30, 2015
 * Description: 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ball_dropper/Heartbeat.h"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/interprocess/sync/scoped_lock.hpp"
#include "boost/shared_ptr.hpp"
#include <vector>
#include "random_vector_analysis.hpp"

#define OP_IDLE 0
#define OP_OPEN_HATCH 1
#define OP_CLOSE_HATCH 2
#define OP_ROTATE 3
#define OP_DRIVE_ON 4
#define OP_DRIVE_OFF 5
#define OP_INJECT 6
#define OP_RELOAD 7
#define OP_FLUSH 8
#define OP_CALIBRATE 100
#define OP_GET_ERROR_STRING 101
#define OP_CLEAR_ERRORS 102
#define OP_EMG_STOP 255


typedef struct Trace_ {
    std::vector<double> values;
} Trace;

typedef struct Operation_ {
    uint8_t opCode;
    boost::shared_ptr<RandomVectorAnalysis> rva;
    std::vector<Trace> traces;
} Operation;

std::vector<Operation> operations;

std::vector<std::string> fieldNames;

double voltage;

void voltageCallback(const std_msgs::Float64& msg)
{
    voltage = msg.data;
    return;
}

uint32_t previousActionStartTime = 0;

void heartbeatCallback(const ball_dropper::Heartbeat& msg)
{
    if (msg.currentOpCode == 0 &&
        msg.opCodeOfLastAction != msg.currentOpCode &&
        msg.actionStartTime != previousActionStartTime
        )
    {
        previousActionStartTime = msg.actionStartTime;

        Trace trace;
        //trace.values.push_back(voltage);

        trace.values.push_back((double)msg.actionDuration);
        trace.values.push_back((double)msg.counterEndVal - msg.counterStartVal);
        trace.values.push_back((double)msg.counterEndVal);
        trace.values.push_back((double)msg.lowestCurrent);
        trace.values.push_back((double)msg.highestCurrent);
        trace.values.push_back((double)msg.totalCurrent);

        int o;
        for (o = 0; o < operations.size(); ++o)
        {
            if (operations[o].opCode == msg.opCodeOfLastAction)
            {
                break;
            }
        }
        if (o == operations.size())
        {
            Operation op;
            op.opCode = msg.opCodeOfLastAction;
            op.rva.reset(new RandomVectorAnalysis(fieldNames));
            operations.push_back(op);
        }
        //printf("Operation %u Distance: %f\n", operations[o].opCode, operations[o].rva->getCovarianceDistance(trace.values));
        bool success = operations[o].rva->addToHistory(trace.values);
        operations[o].traces.push_back(trace);
    }

    return;
}

void printOpTraces()
{
    FILE* file = fopen("/home/ebeachly/OpTraces.csv", "w");
    for (int o = 0; o < operations.size(); ++o )
    {
        printf("\nOperation %u:\n", operations[o].opCode);
        for (int t = 0; t < operations[o].traces.size(); ++t)
        {
            printf("\t");
            fprintf(file, "%u, ", operations[o].opCode);
            for (int v = 0; v < operations[o].traces[t].values.size(); ++v)
            {
                printf("%f ", operations[o].traces[t].values[v]);
                fprintf(file, "%f, ", operations[o].traces[t].values[v]);
            }
            printf("\n");
            fprintf(file, "\n");
        }
    }
    fclose(file);
    return;
}

//Main
int main(int argc, char **argv)
{
    //Create a node
    ros::init(argc, argv, "heartbeat_analyzer_node");
    ros::NodeHandle n;

    ros::Subscriber heartbeatSub = n.subscribe("heartbeat", 100, heartbeatCallback);

    ros::Subscriber voltageSub = n.subscribe("voltage", 1, voltageCallback);

    //fieldNames.push_back("Voltage");
    fieldNames.push_back("Action Duration");
    fieldNames.push_back("Counter Change");
    fieldNames.push_back("Counter Final");
    fieldNames.push_back("Lowest Current");
    fieldNames.push_back("Highest Current");
    fieldNames.push_back("Total Current");

    //Load any operation files
    Operation op3;
    op3.opCode = 3;
    op3.rva.reset(new RandomVectorAnalysis(fieldNames));
    if ( op3.rva->loadHistory("3.tsv") )
    {
        operations.push_back(op3);
    }

    Operation op4;
    op4.opCode = 4;
    op4.rva.reset(new RandomVectorAnalysis(fieldNames));
    if ( op4.rva->loadHistory("4.tsv") )
    {
        operations.push_back(op4);
    }
    
    Operation op5;
    op5.opCode = 5;
    op5.rva.reset(new RandomVectorAnalysis(fieldNames));
    if ( op5.rva->loadHistory("5.tsv") )
    {
        operations.push_back(op5);
    }
    
    Operation op6;
    op6.opCode = 6;
    op6.rva.reset(new RandomVectorAnalysis(fieldNames));
    if ( op6.rva->loadHistory("6.tsv") )
    {
        operations.push_back(op6);
    }
    

    std::vector<std::string> names;
    names.push_back("Math");
    names.push_back("English");
    names.push_back("Art");
    RandomVectorAnalysis rva(names);

    std::vector<double> sample(3,0.0);
    sample[0] = 90; sample[1] = 60; sample[2] = 90;
    rva.addToHistory(sample);
    sample[0] = 90; sample[1] = 90; sample[2] = 30;
    rva.addToHistory(sample);
    sample[0] = 60; sample[1] = 60; sample[2] = 60;
    rva.addToHistory(sample);
    sample[0] = 60; sample[1] = 60; sample[2] = 90;
    rva.addToHistory(sample);
    sample[0] = 30; sample[1] = 30; sample[2] = 30;
    rva.addToHistory(sample);

    printf("%f\n", rva.getGaussianDistance(sample));
    sample[0] = 66; sample[1] = 60; sample[2] = 60;
    printf("%f\n", rva.getGaussianDistance(sample));
    sample[0] = 0; sample[1] = 0; sample[2] = 0;
    printf("%f\n", rva.getGaussianDistance(sample));

    //Handle subscriber and service server callbacks in this thread
    ros::spin();

    //On shutdown, write out the operation histories

    char fileName[256];
    for (int o = 0; o < operations.size(); ++o )
    {
        sprintf(fileName,"%u.tsv", operations[o].opCode);
        operations[o].rva->saveHistory(fileName);
    }

    //printOpTraces();

    return 0;
}

