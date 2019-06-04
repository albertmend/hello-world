//rosrun rosserial_python serial_node.py _port:=tcp _/rosserial_embeddedlinux/tcp_port:=11416 __name:=rosserial_python_laser2

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"
#include "people_msgs/PositionMeasurementArray.h"
#define PI (3.141592653589793)
	
	/*	Detectar la persona por el ancho en cm que tiene a partir del
	 * ángulo y las distancias de inicio y fin de valle, aprox 40 cm.
	 * */
int nmuestras=-1;
int distancia_maxima=-1,distancia_minima=-1; //distancias en mm
int ancho_de_objeto;
int ancho_de_persona=-1; //400 mm
int tolerancia_profundidad_valle=-1; //150 mm
int tolerancia_ancho_valle=-1; //100 mm
int *inicio_valles,*fin_valles; //aquí se guardan los índices donde inician y 
								//donde terminan los valles
int contador_valles=0;
int bandera_dentro_valle=0;

float *Arr;
float resolucion_angular;
float angulo_objeto;
float distancia_promedio_objeto;

ros::Publisher people_measurements_pub_;

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array);
int j,k=0;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "arraySubscriber");
	ros::NodeHandle n;	
	n.getParam("dist_max",distancia_maxima);
	n.getParam("dist_min",distancia_minima);
	n.getParam("ancho_persona",ancho_de_persona);
	n.getParam("tolerancia_profundidad_valle",tolerancia_profundidad_valle);
	n.getParam("tolerancia_ancho_valle",tolerancia_ancho_valle);
	//std::cout<<"distancia máxima(mm): "<<distancia_maxima<< "  distancia minima(mm): "<<distancia_minima<< "  ancho: "<<ancho_de_persona<<"  tolerancia profundidad valle (mm): "<<tolerancia_profundidad_valle<<" tolerancia ancho valle (mm): "<<tolerancia_ancho_valle<<std::endl;
	
	people_measurements_pub_ = n.advertise<people_msgs::PositionMeasurementArray>("people_tracker_measurements", 0);
	ros::Subscriber sub3 = n.subscribe("laser2_msg", 100, arrayCallback);//laser2_msg
	
	ros::spin();
	
	return 0;
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	//printf("------ \n LaserCB\n"); fflush(stdout);
	if (nmuestras == -1){
		nmuestras = array->data.size();
		Arr=(float *) malloc((nmuestras)*sizeof(float));
		inicio_valles=(int *) malloc((nmuestras)*sizeof(int));
		fin_valles=(int *) malloc((nmuestras)*sizeof(int));
		
		//calculamos la resolución angular
		resolucion_angular=240.0/nmuestras; //en grados
		
	}
	
	int j = 0;
	ancho_de_objeto=0;
	contador_valles=0;
	//printf("array->data.size() %d\n",array->data.size());fflush(stdout);
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[j] = *it;
		if(Arr[j]>distancia_minima&&Arr[j]<distancia_maxima){
			//entró al valle
			if(bandera_dentro_valle==0){
				inicio_valles[contador_valles]=j;
				bandera_dentro_valle=1;
			}
		}
		else{
			if (bandera_dentro_valle==1){
				//salió del valle
				fin_valles[contador_valles]=j;
				bandera_dentro_valle=0;
				contador_valles++;
			}
		}
		j++;
	}	
	
	//printf("valles: %d \n", contador_valles);fflush(stdout);
	std::vector<people_msgs::PositionMeasurement> people;
	ros::Time time = ros::Time::now();
	float distancia_promedio_objeto_m;
		//~ //detectamos los valles y su longitud 
 		for(int i=0; i<contador_valles;i++){
			if(fin_valles[i]-inicio_valles[i]>20){
				//printf("\n\nmás de 20 muestras\n");
				//tenemos más de 20 muestras consecutivas, calculamos el ángulo que abarca el objeto
				angulo_objeto=(fin_valles[i]-inicio_valles[i])*resolucion_angular;
				//printf("fin_valles[i]=%i,inicio_valles[i]=%i,resolución angular=%f, ángulo del objeto: %f\n",fin_valles[i],inicio_valles[i],resolucion_angular,angulo_objeto);
				//printf("dir posible: %f\n",-((fin_valles[i]+inicio_valles[i])*0.5*240.0/nmuestras-120));
				//calculamos la distancia promedio a la que se encuentra el objeto
				distancia_promedio_objeto=Arr[inicio_valles[i]];
				//printf("distancia promedio objeto (mm): %f\n",distancia_promedio_objeto);
				//calculamos el ancho del objeto
				ancho_de_objeto=sqrtf(2*powf(distancia_promedio_objeto,2)*(1-cosf(angulo_objeto*PI/180)))	;
				//printf("ancho del objeto (cm): %f \t ancho de una persona (cm): %f a %f\n",(float)ancho_de_objeto/10,(float)(ancho_de_persona-tolerancia_ancho_valle)/10,(float)(ancho_de_persona+tolerancia_ancho_valle)/10);
				//comparamos contra el ancho de una persona, si cae en la tolerancia,
				//detectamos una persona
				if(abs(abs(ancho_de_objeto)-abs(ancho_de_persona))<=tolerancia_ancho_valle){
					//desplegamos la información
					double dir_valle = ((inicio_valles[i]+fin_valles[i])*0.5*240.0/nmuestras-120);
					//printf("Persona detectada en dirección: %f° \t a %f mm",dir_valle,distancia_promedio_objeto);
					distancia_promedio_objeto_m = distancia_promedio_objeto/1000;
				
					//reliability = reliability * other->reliability;
				
					std::ostringstream s;
					s << "people"<< i;
				
					people_msgs::PositionMeasurement pos;
					pos.header.stamp = time;
					pos.header.frame_id = "/base_link";
					pos.name = s.str();
					pos.object_id = s.str();
					pos.pos.x = distancia_promedio_objeto_m*cos(dir_valle*PI/180);
					pos.pos.y = distancia_promedio_objeto_m*sin(dir_valle*PI/180);
					//printf("Persona detectada en x: %f \t y %f\n",pos.pos.x,pos.pos.y);
					//printf("Persona %i detectada en theta: %f \n",i,dir_valle);


					pos.pos.z = 0.0;
					//pos.reliability = reliability;
					//pos.covariance[0] = pow(0.3 / reliability,2.0);
					//pos.covariance[1] = 0.0;
					//pos.covariance[2] = 0.0;
					//pos.covariance[3] = 0.0;
					//pos.covariance[4] = pow(0.3 / reliability,2.0);
					//pos.covariance[5] = 0.0;
					//pos.covariance[6] = 0.0;
					//pos.covariance[7] = 0.0;
					//pos.covariance[8] = 10000.0;
					//pos.initialization = 0;
					people.push_back(pos);  
				}
			}
			
		}
		//fflush(stdout);

	people_msgs::PositionMeasurementArray mes_array;
	mes_array.people = people;
	mes_array.header.stamp = time;
	people_measurements_pub_.publish(mes_array);
	
	return;
}
