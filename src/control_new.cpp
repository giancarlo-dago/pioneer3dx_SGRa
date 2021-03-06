#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Time.h"
#include "tf/tf.h"
#include "boost/thread.hpp"

#define L 0.33												// distanza tra le ruote
#define R 0.1953											// raggio delle ruote
#define MAX_BLOB_SIZE 190.0

using namespace std;

class CONTROLLER_NEW {
	public:
		CONTROLLER_NEW();
		void supervisor();
		void move(float, float);
		void check_obstacles();
		void search_for(int, int&);
		void move_to(int);
		void sleeping(int&);
		void avoid();
		// void check_for_hunger();
		// void collision_detection();
		void feeding(int&);
		void flee();
		void sensor_sub_callback_1( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_2( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_3( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_4( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_5( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_6( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_7( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_8( sensor_msgs::RangeConstPtr );
		void odom_sub_callback( geometry_msgs::PoseConstPtr );
		void red_centering_error_sub_callback( std_msgs::Float32ConstPtr );
		void blue_centering_error_sub_callback( std_msgs::Float32ConstPtr );
		void green_centering_error_sub_callback( std_msgs::Float32ConstPtr );
		void red_blobsize_sub_callback( std_msgs::Float32ConstPtr );		
		void blue_blobsize_sub_callback( std_msgs::Float32ConstPtr );
		void green_blobsize_sub_callback( std_msgs::Float32ConstPtr );
		void run();
	private:
		ros::NodeHandle _nh;
		ros::Publisher  _left_cmd_pub;
		ros::Publisher  _right_cmd_pub;
		ros::Subscriber _sensor_sub_1;
		ros::Subscriber _sensor_sub_2;
		ros::Subscriber _sensor_sub_3;
		ros::Subscriber _sensor_sub_4;
		ros::Subscriber _sensor_sub_5;
		ros::Subscriber _sensor_sub_6;
		ros::Subscriber _sensor_sub_7;
		ros::Subscriber _sensor_sub_8;
		ros::Subscriber _odom_sub;
		ros::Subscriber _red_centering_error_sub;
		ros::Subscriber _red_blobsize_sub;
		ros::Subscriber _blue_centering_error_sub;
		ros::Subscriber _blue_blobsize_sub;
		ros::Subscriber _green_centering_error_sub;
		ros::Subscriber _green_blobsize_sub;
		float _dist [8];					// vettore delle letture dei sonar
		std_msgs::Float32 _wLeft;			// velocità (angolare) ruota sinistra [rad/s]
		std_msgs::Float32 _wRight;			// velocità (angolare) ruota destra	[rad/s]	
		float _yaw;							// orientamento del robot
		float _x;							// posizione lungo x del robot
		float _y;							// posizione lungo y del robot
		float _avoidance_error;				// errore tra direzione attuale e nuova direzione necessaria per evitare ostacoli
		float _red_centering_error;			// errore di orientamento verso il blob di colore rosso
		float _blue_centering_error;		// errore di orientamento verso il blob di colore blue
		float _green_centering_error;		// errore di orientamento verso il blob di colore verde
		float _red_blobsize;				// dimensione del più grande blob di colore rosso
		float _blue_blobsize;				// dimensione del blob di colore blu
		float _green_blobsize;				// dimensione del blob di colore blu
		bool _OBSTACLE_DETECTED;
		bool _RED_DETECTED;
		bool _BLUE_DETECTED;
		bool _GREEN_DETECTED;
		bool _HUNGRY;
		// bool _first_data_arrived;
};



CONTROLLER_NEW::CONTROLLER_NEW() {																		// costruttore: inizializza publishers e subscribers e inizializza variabili della classe
	_left_cmd_pub = _nh.advertise< std_msgs::Float32 > ("/leftwheel_cmd", 0);
	_right_cmd_pub = _nh.advertise< std_msgs::Float32 > ("/rightwheel_cmd", 0);
	_sensor_sub_1 = _nh.subscribe("/sonar1", 0, &CONTROLLER_NEW::sensor_sub_callback_1, this);
	_sensor_sub_2 = _nh.subscribe("/sonar2", 0, &CONTROLLER_NEW::sensor_sub_callback_2, this);
	_sensor_sub_3 = _nh.subscribe("/sonar3", 0, &CONTROLLER_NEW::sensor_sub_callback_3, this);
	_sensor_sub_4 = _nh.subscribe("/sonar4", 0, &CONTROLLER_NEW::sensor_sub_callback_4, this);
	_sensor_sub_5 = _nh.subscribe("/sonar5", 0, &CONTROLLER_NEW::sensor_sub_callback_5, this);
	_sensor_sub_6 = _nh.subscribe("/sonar6", 0, &CONTROLLER_NEW::sensor_sub_callback_6, this);
	_sensor_sub_7 = _nh.subscribe("/sonar7", 0, &CONTROLLER_NEW::sensor_sub_callback_7, this);
	_sensor_sub_8 = _nh.subscribe("/sonar8", 0, &CONTROLLER_NEW::sensor_sub_callback_8, this);
	_odom_sub = _nh.subscribe("/odom", 0, &CONTROLLER_NEW::odom_sub_callback, this);
	_red_centering_error_sub = _nh.subscribe("/centering_error_red" , 0, &CONTROLLER_NEW::red_centering_error_sub_callback, this );
	_blue_centering_error_sub = _nh.subscribe("/centering_error_blue" , 0, &CONTROLLER_NEW::blue_centering_error_sub_callback, this );
	_green_centering_error_sub = _nh.subscribe("/centering_error_green" , 0, &CONTROLLER_NEW::green_centering_error_sub_callback, this );
	_red_blobsize_sub = _nh.subscribe("/blob_size_red" , 0, &CONTROLLER_NEW::red_blobsize_sub_callback, this );
	_blue_blobsize_sub = _nh.subscribe("/blob_size_blue" , 0, &CONTROLLER_NEW::blue_blobsize_sub_callback, this );
	_green_blobsize_sub = _nh.subscribe("/blob_size_green" , 0, &CONTROLLER_NEW::green_blobsize_sub_callback, this );

	// _first_data_arrived = false;
	_OBSTACLE_DETECTED = false;
	_RED_DETECTED = false;
	_HUNGRY = true;
	_red_blobsize = 0.0;
	_blue_blobsize = 0.0;
	_green_blobsize = 0.0;
}



void CONTROLLER_NEW::sensor_sub_callback_1( sensor_msgs::RangeConstPtr distance ) {					// salva info sulle letture di distanza del sonar numero 1
	_dist[0] = distance->range;
	// _first_data_arrived = true;
}

void CONTROLLER_NEW::sensor_sub_callback_2( sensor_msgs::RangeConstPtr distance ) {
	_dist[1] = distance->range;
}

void CONTROLLER_NEW::sensor_sub_callback_3( sensor_msgs::RangeConstPtr distance ) {
	_dist[2] = distance->range;
}

void CONTROLLER_NEW::sensor_sub_callback_4( sensor_msgs::RangeConstPtr distance ) {
	_dist[3] = distance->range;
}

void CONTROLLER_NEW::sensor_sub_callback_5( sensor_msgs::RangeConstPtr distance ) {
	_dist[4] = distance->range;
}

void CONTROLLER_NEW::sensor_sub_callback_6( sensor_msgs::RangeConstPtr distance ) {
	_dist[5] = distance->range;
}

void CONTROLLER_NEW::sensor_sub_callback_7( sensor_msgs::RangeConstPtr distance ) {
	_dist[6] = distance->range;
}

void CONTROLLER_NEW::sensor_sub_callback_8( sensor_msgs::RangeConstPtr distance ) {
	_dist[7] = distance->range;
}

void CONTROLLER_NEW::odom_sub_callback( geometry_msgs::PoseConstPtr pose) {					// salva in _x, _y, _yaw, le info odometriche sulla posizione e l'orientamento (necessario ???)
	_x = pose->position.x;
	_y = pose->position.y;

	tf::Quaternion q(
	pose->orientation.x,
	pose->orientation.y,
	pose->orientation.z,
	pose->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    _yaw = yaw;

	if(_yaw < 0) _yaw += 2*M_PI;
   	if(_yaw > 2*M_PI) _yaw -= 2*M_PI;
}

void CONTROLLER_NEW::red_centering_error_sub_callback( std_msgs::Float32ConstPtr error) {		// comunica di aver trovato il rosso e salva in _red_centering_error l'errore di centramento
	_red_centering_error = error->data;
	if (fabs(_red_centering_error) != 0) _RED_DETECTED=true;
	else _RED_DETECTED = false;
}

void CONTROLLER_NEW::blue_centering_error_sub_callback( std_msgs::Float32ConstPtr error) {		// comunica di aver trovato il rosso e salva in _red_centering_error l'errore di centramento
	_blue_centering_error = error->data;
}

void CONTROLLER_NEW::green_centering_error_sub_callback( std_msgs::Float32ConstPtr error) {		// comunica di aver trovato il rosso e salva in _red_centering_error l'errore di centramento
	_green_centering_error = error->data;
}

void CONTROLLER_NEW::red_blobsize_sub_callback( std_msgs::Float32ConstPtr size) {				// salva in _blobsize le info sulla dimensione del blob più grande del range visivo attuale
	_red_blobsize = size->data;
}

void CONTROLLER_NEW::blue_blobsize_sub_callback( std_msgs::Float32ConstPtr size) {				// salva in _blobsize le info sulla dimensione del blob più grande del range visivo attuale
	_blue_blobsize = size->data;
	if (_blue_blobsize) _BLUE_DETECTED=true;
	else _BLUE_DETECTED = false;
}

void CONTROLLER_NEW::green_blobsize_sub_callback( std_msgs::Float32ConstPtr size) {				// salva in _blobsize le info sulla dimensione del blob più grande del range visivo attuale
	_green_blobsize = size->data;
	if (_green_blobsize) _GREEN_DETECTED=true;
	else _GREEN_DETECTED = false;
}





void CONTROLLER_NEW::flee() {
	float error = 0.0;
	float des_yaw = 0.0;
	float vD, wD;
	float gain1 = 4.0;
	float gain2 = 0.01;
	bool done = false;
	ros::Rate r(100);
	vD = 0.0;
	int k = 0;

	while (fabs(_blue_centering_error) > 5) {					// centramento rispetto al predatore
		if (_HUNGRY) cout << "[ hungry ]    ";
		else cout << "[ not hungry ]    "; 
		cout << "FLEE" << endl;
		wD = gain2 * _blue_centering_error;						
		move(vD, wD);
		r.sleep();															
	}

	des_yaw = _yaw + M_PI;
	error = des_yaw - _yaw;
	if(des_yaw < 0) des_yaw += 2*M_PI;
	if(des_yaw > 2*M_PI) des_yaw -= 2*M_PI;

	while (fabs(error) > 0.05) {								// rotazione di 180 gradi
		if (_HUNGRY) cout << "[ hungry ]    ";
		else cout << "[ not hungry ]    "; 
		cout << "FLEE" << endl;
		error = des_yaw - _yaw;
		wD = gain1 * error;
		move(vD, wD);
		r.sleep();
	}

	while (!done && !_OBSTACLE_DETECTED) {						// vai avanti per 1 metro, se nel frattempo rilevi degli ostacoli esci dal ciclo
		if (_HUNGRY) cout << "[ hungry ]    ";
		else cout << "[ not hungry ]    "; 
		cout << "FLEE" << endl;
		vD = 1.2;
		wD = 0.0;
		move (vD, wD);
		if (k>80) {
			k = 0;
			done = true;
		}
		k++;
		r.sleep();
	}
	move (0.0, 0.0);
}




// void CONTROLLER_NEW::collision_detection() {
	
// }




// void CONTROLLER_NEW::check_for_hunger () {
// 	int time = 0;
// 	ros::Rate r(10);   											// 10 Hz
// 	while (ros::ok()) {
// 		if(_HUNGRY == false) time++;
// 		if (time == 150) {										// dopo 100 decimi di secondo, quindi dopo 10 secondi
// 			_HUNGRY = true;										// ritorno ad avere fame
// 			time = 0;
// 		}
// 		r.sleep();
// 	}
// }




void CONTROLLER_NEW::avoid() {
	float vD;
	float wD;

	if (_avoidance_error>=0) wD = 1-(_avoidance_error/1.5707);		// tanto più velocemente quanto perpendicolarmente sto andando verso il muro
	else wD = -1-(_avoidance_error/1.5707);							// tanto più velocemente quanto perpendicolarmente sto andando verso il muro
	wD = wD*4;														// 4 è solo un fattore moltiplicativo per non farlo ruotare troppo lentamente

	if (fabs(_avoidance_error) > 1.50 ) {							// se sto posizionato quasi parallelamente al muro/ostacolo 
			vD = 0.5;												// mi muovo anche in avanti
			if (_avoidance_error>=0) wD = 0.1;						// e do una velocità angolare (positiva o negativa a seconda del segno dell'errore) un pò più forte in modo da allontanarmi dal muro
			else wD = -0.1;
	}
	else vD = 0.0;													// altrimenti (se sto ancora rivolto verso il muro) giro solamente (altrimenti se avessi anche vD andrei a sbattere)

	move(vD,wD);
}




void CONTROLLER_NEW::sleeping(int& f) {
	move(0.0, 0.0);												// mi fermo (vel lineare e vel angolare a 0)
	if (f>500) {												// se sono passati 10 secondi
		f = 0;
		_HUNGRY = true;										// non sono più affamato
	}
	else f++;
}




void CONTROLLER_NEW::feeding(int& f) {
	move(0.0, 0.0);												// mi fermo (vel lineare e vel angolare a 0)
	if (f>500) {												// se sono passati 10 secondi
		f = 0;
		_HUNGRY = false;										// non sono più affamato
	}
	else f++;
}




void CONTROLLER_NEW::move_to(int color) {
	float centering_error;
	float gain = 0.01;;
	float vD, wD;

	if (color == 1) {											// se devo muovermi verso il rosso
		centering_error = _red_centering_error;
	}
	else if (color == 2) {										// se devo muovermi verso il verde
		centering_error = _green_centering_error;					
	}

	if (fabs(centering_error) > 5) {							// quando l'errore è al di sopra di una certa soglia di errore di "centramento"
		wD = gain * centering_error;							// ruota con una vel. angolare proporzionale all'errore			
		vD = 0.1;												// lo faccio anche un pò muovere in avanti per non farlo oscillare intorno all'errore (necessario? riprova a levarlo)										
	}
	else {														// quando il robot è centrato rispetto al centroide del blob
		vD = 0.5;												// allora muoviti solo in avanti
		wD = 0.0;
	}
	move(vD,wD);
}




void CONTROLLER_NEW::search_for(int color, int& k) {
	float gain = 1.5;
	bool done;
	float vD, wD;
	float LO, HI;
	int t;
	if (color == 1) {						// se cerco il rosso
		LO = -0.5;
		HI = 0.5;
		wD = 1.0;
		vD = 0.6;
		t = 100;
	}
	else if (color == 2) {					// se cerco il verde
		LO = -1.5;					
		HI = 1.5;					
		wD = 0.8;
		vD = 0.4;
		t = 300;
	}
	ros::Rate r(100);

	if (k>t) {																							// ogni time secondi cambio direzione
		cout << "Searching for : changing direction" << endl;
		k = 0;
		done = false;
		srand((unsigned int)time(0));
		float delta_alfa = LO+static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(HI-LO))); 			// genero un nuovo errore di orientamento random compreso tra LO e HI
		while (!done && !_OBSTACLE_DETECTED && !_BLUE_DETECTED) {											// aspetto che il robot venga riorientato (ho messo anche !_OBSTACLE_DETECTED e _BLOB_DETECTED altrimenti si allooppa in questo while anche se rilevo un ostacolo o un predatore)
			if (delta_alfa>0.0) {																			// se l'errore è positivo giro verso destra
				delta_alfa = delta_alfa - wD*0.01;															// decremento l'errore secondo la formula: new_e = old_e - vel.ang. * tempo_trascorso
			}
			else if (delta_alfa<0.0){																		// se l'errore è negativo giro verso sinistra
				delta_alfa = delta_alfa - (-wD)*0.01;														// decremento l'errore secondo la formula: new_e = old_e - vel.ang. * tempo_trascorso
			}
			// if (_yaw >=0 && _yaw <=M_PI) 	wD = -gain * delta_alfa;									// lo faccio ruotare con una velocità angolare proporzionale all'errore
			// else wD = gain * delta_alfa;																	// lo faccio ruotare con una velocità angolare proporzionale all'errore
			if (fabs(delta_alfa)<0.05) done=true;															// quando l'errore è quasi zero esco dal ciclo
			move(0.0,wD);																					// pubblico velocità e velocità angolare
			r.sleep();
		}
	}
	else {
		k++;
		move(vD, 0.0);
	}
}




void CONTROLLER_NEW::check_obstacles() {
	// parametri del robot
    float alfa0 = 3.1415/8.0;									// distanza angolare (in rad) tra un sonar e l'altro
    float alfa[8];												// posizione angolare dei sonar rispetto all'angolo 0
    for (int i=-4; i<0; i++) {
        alfa[i+4] = i*alfa0;
    }
    for (int i=1; i<=4; i++) {
    	alfa[i+3] = i*alfa0;
    }

    // // inizializzazione del thread
	// cout << "CONTROLLER_NEW started" << endl;
	// while( !_first_data_arrived ) sleep(1);
	// ROS_INFO("First data arrived!");
	ros::Rate r(10);
	
	// variabili di controllo
    float current_angle = 0.0;  								// direzione corrente del robot [rad]
    float next_angle = 0.0;										// prossima direzione calcolata sulla base delle distanze [rad]
    float delta_alfa = 0.0;										// differenza tra prossima direzione e direzione corrente [rad]
	
	// obstacle detection
	while( ros::ok() ) {
		float sum_den = 0.0;
		float sum_num = 0.0;
		for (int i=0; i<8; i++) {								// calcolo prossima direzione sulla base delle misure
			sum_den = sum_den + _dist[i];
			sum_num = sum_num + alfa[i]*_dist[i];		
		}
		if (sum_den == 0) {										// se non ci sono rilevazioni da parte dei sonar
			_OBSTACLE_DETECTED = false;
		}
		else {													// se ci sono rilevazioni evita gli ostacoli
			_OBSTACLE_DETECTED = true;
			next_angle = sum_num/sum_den;
			_avoidance_error = next_angle - current_angle;
		}
		r.sleep();	
	}
}




void CONTROLLER_NEW::move(float vD, float wD) {
	_wLeft.data = (2.0*vD - wD*L)/(2.0*R);
	_wRight.data = (2.0*vD + wD*L)/(2.0*R);
	_left_cmd_pub.publish(_wLeft);
	_right_cmd_pub.publish(_wRight);
}




void CONTROLLER_NEW::supervisor() {
	int red = 1;
	int green = 2;
	int k = 0;
	int f = 0;
	int s = 0;

	ros::Rate r(100);
	sleep (2);

	while(ros::ok()) {
		
		if (!_HUNGRY) { 
			cout << "[ not_hungry ]    ";
			if(_OBSTACLE_DETECTED && _green_blobsize < 170) {
				avoid();
				cout << "AVOID" << endl;
			}
			else {
				if(!_GREEN_DETECTED) {
					if (!_BLUE_DETECTED) {
						search_for(green, k);
						cout << "SEARCH_FOR_GREEN" << endl;
					}
					else {
						flee();
						cout << "FLEE" << endl;
					}
				}
				else {
					if (_green_blobsize < MAX_BLOB_SIZE) {
						if (!_BLUE_DETECTED) {
							move_to(green); 
							cout << "MOVE_TO_GREEN" << endl;
						}
						else {
							flee();
							cout << "FLEE" << endl;
						}
					}
					else {
						cout << "SLEEPING" << endl;
						sleeping(s); 
					}
				}
			}
		}
		else {  
			cout << "[ hungry ]    ";
			if(_OBSTACLE_DETECTED && _red_blobsize < 170) {					// _red_blobsize < 170 per dire che quando sto sufficientemente vicino al rosso non lo deve considerare come ostacolo
				avoid(); 
				cout << "AVOID" << endl;
			}
			else {
				if(!_RED_DETECTED) {
					if(!_BLUE_DETECTED) {
						search_for(red, k); 
						cout << "SEARCH_FOR_RED" << endl;
					}
					else {
						flee();
						cout << "FLEE" << endl;
					}
				}
				else {
					if (_red_blobsize < MAX_BLOB_SIZE) {
						if (!_BLUE_DETECTED) {
							move_to(red); 
							cout << "MOVE_TO_RED" << endl;
						}
						else {
							flee();
							cout << "FLEE" << endl;
						}
					}
					else {
						cout << "FEEDING" << endl;
						feeding(f); 
					}
				}
			}
		}



		// if(!_GREEN_DETECTED) {
		// 	cout << "SEARCH_FOR" << endl;
		// 	search_for(green, k);
		// }	
		// else {
		// 	if (_green_blobsize < MAX_BLOB_SIZE) {
		// 		cout << "MOVE_TO" << endl;
		// 		move_to(green);
		// 	}
		// 	else {
		// 		cout << "SLEEPING" << endl;
		// 		move (0.0, 0.0);
		// 	}
		// }





		// if(!_RED_DETECTED) {
		// 	cout << "SEARCH_FOR" << endl;
		// 	search_for(red, k);
		// }	
		// else {
			// if (_red_blobsize < MAX_BLOB_SIZE) {
			// 	cout << "MOVE_TO" << endl;
			// 	move_to(red);
			// }
			// else {
			// 	cout << "FEEDING" << endl;
			// 	feeding(f); 
			// }
		// }





		// if (_BLUE_DETECTED)	{
		// 	cout << "FLEE" << endl;
		// 	flee();
		// }
		// else {
		// 	cout << "STOP" << endl;
		// 	move(0.0, 0.0);
		// }
		


		// if (_OBSTACLE_DETECTED)	{
		// 	cout << "AVOIDING" << endl;
		// 	avoid();
		// }
		// else {
		// 	cout << "STOP" << endl;
		// 	move(0.0, 0.0);
		// }



		r.sleep();
	}
}




void CONTROLLER_NEW::run() {
	boost::thread check_obstacles_t( &CONTROLLER_NEW::check_obstacles, this );	
	boost::thread supervisor_t( &CONTROLLER_NEW::supervisor, this );
	// boost::thread check_for_hunger_t( &CONTROLLER_NEW::check_for_hunger, this );
	ros::spin();
}




int main (int argc, char** argv) {
	ros::init(argc, argv, "CONTROLLER_NEW");
	CONTROLLER_NEW ctrl;
	ctrl.run();
	return 0;
}
