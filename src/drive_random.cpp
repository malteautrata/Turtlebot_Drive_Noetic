#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/Header.h"
#include <time.h>       /* time_t, struct tm, difftime, time, mktime */
#include <thread>   
#include <algorithm> 
#include <math.h>         


// ein Publisher um die Geschwindigkeit des Roboters zu setzen
ros::Publisher pub;

// ein Subscriber für das Topic /scan, um die Sensordaten des Roboters aufzurufen
ros::Subscriber subScan;

// ein Subscriber für das Topic /map, um auf die Karten Daten zuzugreifen
ros::Subscriber subMap;

// eine Boolean-Variable, ob der Roboter sich drehen muss
bool shouldDodge = false;

// eine Boolean-Variable, ob der Roboter sich im Moment dreht um ein Hindernis auszuweichen
bool isTurning;

// eine Boolean-Variable, ob der Roboter sich links herum dreht
bool turnLeft;


double Ngray = 0;


// eine zufällige Boolean-Variable wird zurückgegeben
bool randomBool()
{
    return rand() % 2;
}

// alle Werte des Roboters auf 0 setzen, um ihn zum stehen zu bringen
void cleanShutdown()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;
    pub.publish(twist);
}

// eine Callback Methode, die aufgerufen wird, wenn sich die Karten Daten ändern
void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    nav_msgs::MapMetaData info = msg->info;
    int mistakes = 0;
    for(int i=0; i<info.height*info.width; i++)
    {
      //Checken ob der Rahmen geschlossen ist, dadurch, das abgefragt wird, ob neben Pixeln die als grau gekennzeichnet sind(0) Pixel sind, die unbekannt sind(-1). 
        if(double((msg->data[i])==0)&&(double(msg->data[i+1])==-1))
        {
            //Rechts
            //std::cout<<"Karte nicht vollständig!"<<std::endl;

            mistakes++;
        }
        if(double((msg->data[i])==0)&&(double(msg->data[i+1])==-1))
        {
            //Links
            //std::cout<<"Karte nicht vollständig!"<<std::endl;
            mistakes++;
        }
         if(i>info.width)
        {
        if(double((msg->data[i])==0)&&(double(msg->data[i+info.width])==-1))
            {
                //Oben
                //std::cout<<"Karte nicht vollständig!"<<std::endl;
                mistakes++;
            }
        }
        if(i<info.height*info.width-info.width)
        {
            if(double((msg->data[i])==0)&&(double(msg->data[i-info.width])==-1))
            {
                //Unten
                //std::cout<<"Karte nicht vollständig!"<<std::endl;
                
                mistakes++;
            }
        }
        
    }
    std::cout<< "Map Fehler:" << mistakes << std::endl;

    if (mistakes < 30)
    {
        std::cout<<"Karte vollständig"<<std::endl;
        system("rosrun map_server map_saver -f ~/map");
        cleanShutdown();
        exit(0);

        std::cout<<Ngray<<std::endl;
        Ngray=0; 
    }  
}

// die Methode wird aufgerufen, wenn Sensordaten geliefert werden, hier passiert das reagieren des Roboters auf Hindernisse
void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ein Objekt der Klasse Twist wird instanziiert; dieses besitzt Variablen, welche die Geschwinigkeiten entlang der Achsen festlegen
    geometry_msgs::Twist twist;

    /*
    Herausfinden wieviele Daten im angle[] array gespeichert sind: von 0 bis 359
    int num_readings = std::ceil( (msg->angle_max - msg->angle_min) / msg->angle_increment );
    std::cout << num_readings << std::endl;
    std::cout << "Range 350:" <<  msg->ranges[350] << std::endl;

    Herausfinden welche bei der Ausweichung sinnvoll sind
    */
    /*
    for (int i = 0; i <= 359; i++)
    {
        if (msg->ranges[i] <= 0.3 && msg->ranges[i] >= 0.00001)
        {
            std::cout << i << std::endl;
        } 
    }*/
    //std::cout << "new" << std::endl;
    
    // es wird davon ausgegangen, dass sich der Roboter zunächst nicht dreht
    shouldDodge = false;

    // die Hindernisse im Bereich 1° bis 45° werden gescannt: wenn sich ein Hindernis zu nah befindet, wird shouldDodge auf true gesetzt
    for(int i = 135; i <= 179; i++)
    {
        if (msg->ranges[i] <= 0.2 && msg->ranges[i] > 0.0000001)
        {
            shouldDodge = true;
        }
    }

    // die Hindernisse im Bereich 315° bis 359 werden gescannt: wenn sich ein Hindernis zu nah befindet, wird shouldDodge auf true gesetzt
    for(int i = 181; i <= 225; i++)
    {
        if (msg->ranges[i] <= 0.2 && msg->ranges[i] > 0.000001)
        {
            shouldDodge = true;
        }
    }

    // die Hindernisse direkt vor dem Roboter werden mit einem größeren Abstand (40 cm) abgefragt
    if (msg->ranges[180] <= 0.4 && msg->ranges[180] > 0.0000001)
    {
        shouldDodge = true;
    }

    // wenn der Roboter ausweichen soll
    if(shouldDodge)
    {
        // die Fahrtrichtungen werden auf 0 gestellt, damit der Roboter nicht weiter fährt
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0;

        // wenn der Roboter sich noch im Ausweich-Manöver befindet, soll die Richtung nicht geändert werden, ansonsten wird die Ausweichrichtung zufällig bestimmt 
        if (isTurning)
        {

        } else
        {
           turnLeft = randomBool();
        }
       
        // Der Roboter befindet sich im Ausweich-Manöver
        isTurning = true;

        // das Obejtk twist bekommt die Drehgeschwindigkeit festgelegt
        if(turnLeft)
        {
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 2.0;

        } else 
        {
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -2.0;

        }
    }

    // es befindet sich kein Hindernis vor dem Roboter; das Objekt twist bekommt nur eine Geschwindigkeit in x-Richtung festgelegt
    else 
    {
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0;
        twist.linear.x = 0.3; twist.linear.y = 0.0; twist.linear.z = 0.0;
        isTurning = false;
    }

    // das Objekt twist wird in das Topic /cmd_vel veröffentlicht
    pub.publish(twist);
}

// die ROS-Node turtlebot3_slam soll gestartet werden, um die Karte des Roboters zu starten
void launchSlam()
{
    system("roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping");
}

int main(int argc, char** argv){
    time_t seconds;
    time(&seconds);
    srand((unsigned int) seconds);
    ros::init(argc, argv, "drive_random");
    ros::NodeHandle n;

    // Der Publisher und die Subscriber bekommen die jeweiligen Topics und Callback-Funktionen zugewiesen
    pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    subScan = n.subscribe("/scan", 10, callback);
    subMap = n.subscribe("/map", 1, callbackMap);

    // die Methode launchSlam wird auf einen neuen Thread geöffnet
    std::thread slamThread (launchSlam); 

    while(ros::ok())
    {
        ros::spin();
    }
 }
