#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Time.h"
#include "boost/thread.hpp"

#define L 0.33												// distanza tra le ruote
#define R 0.1953											// raggio delle ruote
#define MAX_BLOB_SIZE 190.0

using namespace std;

class CONTROLLER {
	public:
		CONTROLLER();
		void supervisor();
		void move(float, float);
		void check_obstacles();
		void wander(int&);
		void avoid();
		void check_for_hunger();
		void search_for_red(int &);
		// void collision_detection();
		void move_to_red();
		void feeding(int&);
		void sensor_sub_callback_1( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_2( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_3( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_4( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_5( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_6( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_7( sensor_msgs::RangeConstPtr );
		void sensor_sub_callback_8( sensor_msgs::RangeConstPtr );
		void odom_sub_callback( geometry_msgs::PoseConstPtr );
		void centering_error_sub_callback( std_msgs::Float32ConstPtr );
		void blobsize_sub_callback( std_msgs::Float32ConstPtr );
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
		ros::Subscriber _centering_error_sub;
		ros::Subscriber _blobsize_sub;
		// bool _first_data_arrived;
		float _dist [8];					// vettore delle letture dei sonar
		std_msgs::Float32 _wLeft;			// velocità (angolare) ruota sinistra [rad/s]
		std_msgs::Float32 _wRight;			// velocità (angolare) ruota destra	[rad/s]	
		float _yaw;							// orientamento del robot
		float _x;							// posizione lungo x del robot
		float _y;							// posizione lungo y del robot
		float _centering_error;				// errore di orientamento verso il blob di colore
		float _avoidance_error;				// errore tra direzione attuale e nuova direzione necessaria per evitare ostacoli
		float _blobsize;					// dimensione del più grande blob di colore
		bool _OBSTACLE_DETECTED;
		bool _RED_DETECTED;
		bool _HUNGRY;
};



CONTROLLER::CONTROLLER() {																		// costruttore: inizializza publishers e subscribers e inizializza variabili della classe
	_left_cmd_pub = _nh.advertise< std_msgs::Float32 > ("/leftwheel_cmd", 0);
	_right_cmd_pub = _nh.advertise< std_msgs::Float32 > ("/rightwheel_cmd", 0);
	_sensor_sub_1 = _nh.subscribe("/sonar1", 0, &CONTROLLER::sensor_sub_callback_1, this);
	_sensor_sub_2 = _nh.subscribe("/sonar2", 0, &CONTROLLER::sensor_sub_callback_2, this);
	_sensor_sub_3 = _nh.subscribe("/sonar3", 0, &CONTROLLER::sensor_sub_callback_3, this);
	_sensor_sub_4 = _nh.subscribe("/sonar4", 0, &CONTROLLER::sensor_sub_callback_4, this);
	_sensor_sub_5 = _nh.subscribe("/sonar5", 0, &CONTROLLER::sensor_sub_callback_5, this);
	_sensor_sub_6 = _nh.subscribe("/sonar6", 0, &CONTROLLER::sensor_sub_callback_6, this);
	_sensor_sub_7 = _nh.subscribe("/sonar7", 0, &CONTROLLER::sensor_sub_callback_7, this);
	_sensor_sub_8 = _nh.subscribe("/sonar8", 0, &CONTROLLER::sensor_sub_callback_8, this);
	_odom_sub = _nh.subscribe("/odom", 0, &CONTROLLER::odom_sub_callback, this);
	_centering_error_sub = _nh.subscribe("/centering_error" , 0, &CONTROLLER::centering_error_sub_callback, this );
	_blobsize_sub = _nh.subscribe("/blob_size" , 0, &CONTROLLER::blobsize_sub_callback, this );

	// _first_data_arrived = false;
	_OBSTACLE_DETECTED = false;
	_RED_DETECTED = false;
	_HUNGRY = false;
	_blobsize = 0.0;
}



void CONTROLLER::sensor_sub_callback_1( sensor_msgs::RangeConstPtr distance ) {					// salva info sulle letture di distanza del sonar numero 1
	_dist[0] = distance->range;
	// _first_data_arrived = true;
}

void CONTROLLER::sensor_sub_callback_2( sensor_msgs::RangeConstPtr distance ) {
	_dist[1] = distance->range;
}

void CONTROLLER::sensor_sub_callback_3( sensor_msgs::RangeConstPtr distance ) {
	_dist[2] = distance->range;
}

void CONTROLLER::sensor_sub_callback_4( sensor_msgs::RangeConstPtr distance ) {
	_dist[3] = distance->range;
}

void CONTROLLER::sensor_sub_callback_5( sensor_msgs::RangeConstPtr distance ) {
	_dist[4] = distance->range;
}

void CONTROLLER::sensor_sub_callback_6( sensor_msgs::RangeConstPtr distance ) {
	_dist[5] = distance->range;
}

void CONTROLLER::sensor_sub_callback_7( sensor_msgs::RangeConstPtr distance ) {
	_dist[6] = distance->range;
}

void CONTROLLER::sensor_sub_callback_8( sensor_msgs::RangeConstPtr distance ) {
	_dist[7] = distance->range;
}

void CONTROLLER::odom_sub_callback( geometry_msgs::PoseConstPtr pose) {					// salva in _x, _y, _yaw, le info odometriche sulla posizione e l'orientamento (necessario ???)
	_yaw = pose->orientation.z;
	_x = pose->position.x;
	_y = pose->position.y;
}

void CONTROLLER::centering_error_sub_callback( std_msgs::Float32ConstPtr error) {		// comunica di aver trovato il rosso e salva in _centering_error l'errore di centramento
	_centering_error = error->data;
	if (fabs(_centering_error) != 0) _RED_DETECTED=true;
	else _RED_DETECTED = false;
}

void CONTROLLER::blobsize_sub_callback( std_msgs::Float32ConstPtr size) {				// salva in _blobsize le info sulla dimensione del blob più grande del range visivo attuale
	_blobsize = size->data;
}







void CONTROLLER::feeding(int& f) {
	move(0.0, 0.0);												// mi fermo (vel lineare e vel angolare a 0)
	if (f>1000) {												// se sono passati 10 secondi
		f = 0;
		_HUNGRY = false;										// non sono più affamato
	}
	else f++;
}



void CONTROLLER::move_to_red() {
	float gain = 0.01;
	float vD;
	float wD;
	if (fabs(_centering_error) > 5) {							// quando l'errore è al di sopra di una certa soglia di errore di "centramento"
		wD = gain * _centering_error;							// ruota con una vel. angolare proporzionale all'errore			
		vD = 0.1;												// lo faccio anche un pò muovere in avanti per non farlo oscillare intorno all'errore (necessario? riprova a levarlo)										
	}
	else {														// quando il robot è centrato rispetto al centroide del blob
		vD = 0.5;												// allora muoviti solo in avanti
		wD = 0.0;
	}
	move(vD,wD);
}



// void CONTROLLER::collision_detection() {
	
// }



void CONTROLLER::search_for_red(int &k) {

	float LO = -0.5;
	float HI = 0.5;
	bool done;
	float gain = 1.5;
	float vD;
	float wD;
	ros::Rate r(100);
	
	if (k>100) {																					// ogni 1 secondi cambio direzione
		cout << "Searching for red: changing direction" << endl;
		k = 0;
		done = false;
		srand((unsigned int)time(0));
		float new_alfa = LO+static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(HI-LO))); 		// genero un nuovo orientamento random
		float delta_alfa = new_alfa;
		while (!done && !_OBSTACLE_DETECTED) {											// aspetto che il robot venga riorientato
			// delta_alfa = new_alfa - _yaw;					// calcolo il nuovo errore generatosi
			// if(fabs(delta_alfa) > 1)
            //     delta_alfa = delta_alfa- 2*((delta_alfa>0)?1:-1);
			// vD = 0.0;											// fermo il robot
			// if (_yaw >=-0.5 && _yaw <=0.5) 	wD = -gain * delta_alfa;								// lo faccio ruotare con una velocità angolare proporzionale all'errore
			// if (_yaw <-0.5 && _yaw >0.5) 	wD = gain * delta_alfa;								// lo faccio ruotare con una velocità angolare proporzionale all'errore
			// if (fabs(delta_alfa)<0.1 || _RED_DETECTED) done=true;				// quando l'errore è quasi zero esco dal ciclo
			// move(vD,wD);										// pubblico velocità e velocità angolare
			// r.sleep();

			if (delta_alfa>0.0) {
				wD = 1;
				delta_alfa = delta_alfa - wD*0.01;
			}
			else if (delta_alfa<0.0){
				wD = -1;
				delta_alfa = delta_alfa - wD*0.01;
			}
			vD = 0.0;											// fermo il robot
			if (fabs(delta_alfa)<0.01) done=true;				// quando l'errore è quasi zero esco dal ciclo
			move(vD,wD);										// pubblico velocità e velocità angolare
			r.sleep();
		}
	}
	else {
		vD = 0.6;												// nel restante tempo vado diritto 
		wD = 0.0;
		k++;
		move(vD,wD);
	}
}




void CONTROLLER::check_for_hunger () {
	int time = 0;
	ros::Rate r(10);   											// 10 Hz
	while (ros::ok()) {
		if(_HUNGRY == false) time++;
		if (time == 100) {										// dopo 100 decimi di secondo, quindi dopo 10 secondi
			_HUNGRY = true;										// ritorno ad avere fame
			time = 0;
		}
		r.sleep();
	}
}




void CONTROLLER::avoid() {
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



void CONTROLLER::wander(int &k) {
	float LO = -1.57;																						// corrisponde a -pi_greco/2
	float HI = 1.57;																						// corrisponde a + pi_greco/2
	bool done;
	// float gain = 1.5;
	float vD;
	float wD;
	ros::Rate r(100);

	if (k>300) {																							// ogni 5 secondi cambio direzione
		cout << "Wandering: changing direction" << endl;
		k = 0;
		done = false;
		srand((unsigned int)time(0));
		float delta_alfa = LO+static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(HI-LO))); 			// genero un nuovo errore di orientamento random compreso tra LO e HI
		while (!done && !_OBSTACLE_DETECTED) {																// aspetto che il robot venga riorientato (ho messo anche !_OBSTACLE_DETECTED altrimenti si allooppa in questo while anche se rilevo un ostacolo)
			vD = 0.0;																						// fermo il robot
			if (delta_alfa>0.0) {																			// se l'errore è positivo giro verso destra
				wD = 0.8;																					// con questa vel. angolare
				delta_alfa = delta_alfa - wD*0.01;															// decremento l'errore secondo la formula: new_e = old_e - vel.ang. * tempo_trascorso
			}
			else if (delta_alfa<0.0){																		// se l'errore è negativo giro verso sinistra
				wD = -0.8;																					// con questa vel_angolare
				delta_alfa = delta_alfa - wD*0.01;															// decremento l'errore secondo la formula: new_e = old_e - vel.ang. * tempo_trascorso
			}
			// if (_yaw >=-0.5 && _yaw <=0.5) 	wD = -gain * delta_alfa;				// lo faccio ruotare con una velocità angolare proporzionale all'errore
			// if (_yaw <-0.5 && _yaw >0.5) 	wD = gain * delta_alfa;					// lo faccio ruotare con una velocità angolare proporzionale all'errore
			if (fabs(delta_alfa)<0.05) done=true;															// quando l'errore è quasi zero esco dal ciclo
			move(vD,wD);																					// pubblico velocità e velocità angolare
			r.sleep();
		}
	}
	else {
		vD = 0.4;																							// nel restante tempo vado diritto 
		wD = 0.0;
		k++;
		move(vD,wD);
	}

}



void CONTROLLER::check_obstacles() {
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
	// cout << "Controller started" << endl;
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



void CONTROLLER::move(float vD, float wD) {
	_wLeft.data = (2.0*vD - wD*L)/(2.0*R);
	_wRight.data = (2.0*vD + wD*L)/(2.0*R);
	_left_cmd_pub.publish(_wLeft);
	_right_cmd_pub.publish(_wRight);
}



void CONTROLLER::supervisor() {
	ros::Rate r(100);
	int k = 0;
	int f = 0;

	while(ros::ok()) {
		if (!_HUNGRY) { 
			cout << "[ not_hungry ]    ";
			if(_OBSTACLE_DETECTED) {
				avoid();
				cout << "AVOID" << endl;
			}
			else {
				wander(k);
				cout << "WANDER" << endl;
			}
		}
		else {  
			cout << "[ hungry ]    ";
			if(_OBSTACLE_DETECTED && _blobsize < 170) {					// _blobsize < 170 per dire che quando sto sufficientemente vicino al rosso non lo deve considerare come ostacolo
				avoid(); 
				cout << "AVOID" << endl;
			}
			else {
				if(!_RED_DETECTED) {
					search_for_red(k); 
					cout << "SEARCH_FOR_RED" << endl;
				}
				else {
					if (_blobsize < MAX_BLOB_SIZE) {
						move_to_red(); 
						cout << "MOVE_TO_RED" << endl;
					}
					else {
						cout << "FEEDING" << endl;
						feeding(f); 
					}
				}
			}
		}

		// move_to_red();

		r.sleep();
	}
}



void CONTROLLER::run() {
	boost::thread check_obstacles_t( &CONTROLLER::check_obstacles, this );	
	boost::thread supervisor_t( &CONTROLLER::supervisor, this );
	boost::thread check_for_hunger_t( &CONTROLLER::check_for_hunger, this );
	ros::spin();
}



int main (int argc, char** argv) {
	ros::init(argc, argv, "controller");
	CONTROLLER ctrl;
	ctrl.run();
	return 0;
}





// #include "ros/ros.h"
// #include "std_msgs/Float32.h"
// #include "sensor_msgs/Range.h"
// #include "geometry_msgs/Pose.h"
// #include "std_msgs/Time.h"
// #include "boost/thread.hpp"

// #define L 0.33
// #define R 0.1953

// using namespace std;

// class CONTROLLER {
// 	public:
// 		CONTROLLER();
// 		void avoid();
// 		void wander(int&, float&, float&, float&);
// 		void collision_detection();
// 		void move(float, float);
// 		void move_to_red();
// 		void sensor_sub_callback_1( sensor_msgs::RangeConstPtr );
// 		void sensor_sub_callback_2( sensor_msgs::RangeConstPtr );
// 		void sensor_sub_callback_3( sensor_msgs::RangeConstPtr );
// 		void sensor_sub_callback_4( sensor_msgs::RangeConstPtr );
// 		void sensor_sub_callback_5( sensor_msgs::RangeConstPtr );
// 		void sensor_sub_callback_6( sensor_msgs::RangeConstPtr );
// 		void sensor_sub_callback_7( sensor_msgs::RangeConstPtr );
// 		void sensor_sub_callback_8( sensor_msgs::RangeConstPtr );
// 		void odom_sub_callback( geometry_msgs::PoseConstPtr );
// 		void centering_error_sub_callback( std_msgs::Float32ConstPtr );
// 		void run();
// 	private:
// 		ros::NodeHandle _nh;
// 		ros::Publisher  _left_cmd_pub;
// 		ros::Publisher  _right_cmd_pub;
// 		ros::Subscriber _sensor_sub_1;
// 		ros::Subscriber _sensor_sub_2;
// 		ros::Subscriber _sensor_sub_3;
// 		ros::Subscriber _sensor_sub_4;
// 		ros::Subscriber _sensor_sub_5;
// 		ros::Subscriber _sensor_sub_6;
// 		ros::Subscriber _sensor_sub_7;
// 		ros::Subscriber _sensor_sub_8;
// 		ros::Subscriber _odom_sub;
// 		ros::Subscriber _centering_error_sub;
// 		bool _first_data_arrived;
// 		float _dist [8];					// vettore delle letture dei sonar
// 		float wD = 0.0;						// velocità angolare robot desiderata
// 		float vD = 0.3;						// velocità lineare robot desiderata
// 		std_msgs::Float32 _wLeft;			// velocità (angolare) ruota sinistra [rad/s]
// 		std_msgs::Float32 _wRight;			// velocità (angolare) ruota destra	[rad/s]	
// 		float _yaw;							// orientamento del robot
// 		float _x;							// posizione lungo x del robot
// 		float _y;							// posizione lungo y del robot
// 		float _centering_error;				// errore di orientamento verso il blob di colore
// };

// CONTROLLER::CONTROLLER() {
// 	_left_cmd_pub = _nh.advertise< std_msgs::Float32 > ("/leftwheel_cmd", 0);
// 	_right_cmd_pub = _nh.advertise< std_msgs::Float32 > ("/rightwheel_cmd", 0);
// 	_sensor_sub_1 = _nh.subscribe("/sonar1", 0, &CONTROLLER::sensor_sub_callback_1, this);
// 	_sensor_sub_2 = _nh.subscribe("/sonar2", 0, &CONTROLLER::sensor_sub_callback_2, this);
// 	_sensor_sub_3 = _nh.subscribe("/sonar3", 0, &CONTROLLER::sensor_sub_callback_3, this);
// 	_sensor_sub_4 = _nh.subscribe("/sonar4", 0, &CONTROLLER::sensor_sub_callback_4, this);
// 	_sensor_sub_5 = _nh.subscribe("/sonar5", 0, &CONTROLLER::sensor_sub_callback_5, this);
// 	_sensor_sub_6 = _nh.subscribe("/sonar6", 0, &CONTROLLER::sensor_sub_callback_6, this);
// 	_sensor_sub_7 = _nh.subscribe("/sonar7", 0, &CONTROLLER::sensor_sub_callback_7, this);
// 	_sensor_sub_8 = _nh.subscribe("/sonar8", 0, &CONTROLLER::sensor_sub_callback_8, this);
// 	_odom_sub = _nh.subscribe("/odom", 0, &CONTROLLER::odom_sub_callback, this);
// 	_centering_error_sub = _nh.subscribe("/centering_error" , 0, &CONTROLLER::centering_error_sub_callback, this );
// 	_first_data_arrived = false;
// }

// void CONTROLLER::sensor_sub_callback_1( sensor_msgs::RangeConstPtr distance ) {
// 	_dist[0] = distance->range;
// 	_first_data_arrived = true;
// }

// void CONTROLLER::sensor_sub_callback_2( sensor_msgs::RangeConstPtr distance ) {
// 	_dist[1] = distance->range;
// }

// void CONTROLLER::sensor_sub_callback_3( sensor_msgs::RangeConstPtr distance ) {
// 	_dist[2] = distance->range;
// }

// void CONTROLLER::sensor_sub_callback_4( sensor_msgs::RangeConstPtr distance ) {
// 	_dist[3] = distance->range;
// }

// void CONTROLLER::sensor_sub_callback_5( sensor_msgs::RangeConstPtr distance ) {
// 	_dist[4] = distance->range;
// }

// void CONTROLLER::sensor_sub_callback_6( sensor_msgs::RangeConstPtr distance ) {
// 	_dist[5] = distance->range;
// }

// void CONTROLLER::sensor_sub_callback_7( sensor_msgs::RangeConstPtr distance ) {
// 	_dist[6] = distance->range;
// }

// void CONTROLLER::sensor_sub_callback_8( sensor_msgs::RangeConstPtr distance ) {
// 	_dist[7] = distance->range;
// }

// void CONTROLLER::odom_sub_callback( geometry_msgs::PoseConstPtr pose) {
// 	_yaw = pose->orientation.z;
// 	_x = pose->position.x;
// 	_y = pose->position.y;
// }

// void CONTROLLER::centering_error_sub_callback( std_msgs::Float32ConstPtr error) {
// 	_centering_error = error->data;
// }

// void CONTROLLER::move_to_red() {
// 	ros::Rate r(100);
// 	float gain = 0.05;
// 	float vD;
// 	float wD;

// 	while (ros::ok()) {	
// 		if (fabs(_centering_error) > 1) {
// 			vD = 0.0;
// 			wD = gain * _centering_error;
// 		}
// 		else {
// 			// vD = 0.3;
// 			// wD = 0.0;
// 		}
// 		move(vD,wD);
// 		r.sleep();
// 	}
// }

// void CONTROLLER::wander(int &k, float &delta_alfa, float &vD, float &wD) {
// 	float HI = 1;
// 	float LO = -1;
// 	bool done = true;
// 	float gain = 0.8;
// 	ros::Rate r(100);

// 	if (k==500) {												// ogni 5 secondi cambio direzione
// 		cout << "Wandering: changing direction" << endl;
// 		k = 0;
// 		done = false;
// 		float new_alfa = LO+static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(HI-LO))); 		// genero un nuovo orientamento random
// 		while (!done) {											// aspetto che il robot venga riorientato
// 			delta_alfa = new_alfa - _yaw;						// calcolo il nuovo errore generatosi
// 			vD = 0.0;											// fermo il robot
// 			wD = gain * delta_alfa;								// lo faccio ruotare con una velocità angolare proporzionale all'errore
// 			if (fabs(delta_alfa)<0.1) done=true;				// quando l'errore è quasi zero esco dal ciclo
// 			move(vD,wD);										// pubblico velocità e velocità angolare
// 			r.sleep();
// 		}
// 	}
// 	vD = 0.3;													// nel restante tempo vado diritto 
// 	wD = 0.0;
// 	k++;
// 	move(vD,wD);
// 	cout << "Wandering" << endl;
	
// 	// float new_alfa;
// 	// if (rotating == false) {
// 	// 	vD = 0.3;
// 	// 	wD = 0.0;
// 	// 	k++;			
// 	// }
// 	// else if (rotating == true && delta_alfa>0.0) {
// 	// 	delta_alfa = new_alfa - _yaw;
// 	// 	// delta_alfa = delta_alfa - wD*0.02;			//???
// 	// 	if (delta_alfa < 0.0) rotating=false;
// 	// }
// 	// else if (rotating == true && delta_alfa<0.0){
// 	// 	delta_alfa = new_alfa - _yaw;
// 	// 	// delta_alfa = delta_alfa - wD*0.01;
// 	// 	if (delta_alfa > 0.0) rotating=false;
// 	// }
// 	// if (k==300) {																			// ogni 3 secondi
// 	// 	new_alfa = LO+static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(HI-LO))); 		// genero un nuovo errore random di orientamento
// 	// 	delta_alfa = new_alfa - _yaw;
// 	// 	cout << "yaw " <<_yaw <<  "new_alfa " << new_alfa << " delta: " << delta_alfa << endl;
// 	// 	vD = 0.0;																				// fermo il robot
// 	// 	if (delta_alfa>0) wD = 0.5;																// se l'errore è positivo giro verso destra con vel.ang. 0.5
// 	// 	else wD = -0.5;																			// altrimenti giro verso sinistra
// 	// 	k = 0;																					// riazzero il contatore del tempo
// 	// 	rotating = true;																		// alzo il flag che dice che sto ruotando
// 	// }
// }

// void CONTROLLER::collision_detection() {
	
// }


// void CONTROLLER::avoid() {

// 	// parametri del robot
//     float alfa0 = 3.1415/8.0;									// distanza angolare (in rad) tra un sonar e l'altro
//     float alfa[8];												// posizione angolare dei sonar rispetto all'angolo 0
//     for (int i=-4; i<0; i++) {
//         alfa[i+4] = i*alfa0;
//     }
//     for (int i=1; i<=4; i++) {
//     	alfa[i+3] = i*alfa0;
//     }

//     // inizializzazione del thread
// 	cout << "Controller started" << endl;
// 	while( !_first_data_arrived ) sleep(1);
// 	ROS_INFO("First data arrived!");
// 	ros::Rate r(100);
	
// 	// variabili di controllo
//     float current_angle = 0.0;  								// direzione corrente del robot [rad]
//     float next_angle = 0.0;										// prossima direzione calcolata sulla base delle distanze [rad]
//     float delta_alfa = 0.0;										// differenza tra prossima direzione e direzione corrente [rad]
// 	int k = 0;
// 	float kp_o = 1.5;											// guadagno proporzionale
	
// 	//wandering & avoiding
// 	while( ros::ok() ) {
// 		float sum_den = 0.0;
// 		float sum_num = 0.0;
// 		for (int i=0; i<8; i++) {								// calcolo prossima direzione sulla base delle misure
// 			sum_den = sum_den + _dist[i];
// 			sum_num = sum_num + alfa[i]*_dist[i];		
// 		}
// 		if (sum_den == 0) {										// se non ci sono rilevazioni da parte dei sonar
// 			wander(k, delta_alfa, vD, wD);						// vaga per il mondo
// 		}
// 		else {													// se ci sono rilevazioni evita gli ostacoli
// 			cout << "Avoiding obstacles" << endl;
// 			vD = 0.3;
// 			next_angle = sum_num/sum_den;
// 			delta_alfa = next_angle - current_angle;
// 			// wD = kp_o * (delta_alfa);						// controllore proporzionale (non funziona bene)
// 			if (delta_alfa>=0) wD = 1-(delta_alfa/1.5707);		// funziona meglio questa soluzione qua
// 			else wD = -1-(delta_alfa/1.5707);
// 			wD = wD*4;
// 			move(vD,wD);
// 		}
// 		r.sleep();	
// 	}
// }

// void CONTROLLER::move(float vD, float wD) {
// 	_wLeft.data = (2.0*vD - wD*L)/(2.0*R);
// 	_wRight.data = (2.0*vD + wD*L)/(2.0*R);
// 	_left_cmd_pub.publish(_wLeft);
// 	_right_cmd_pub.publish(_wRight);
// }

// void CONTROLLER::run() {
// 	// boost::thread move_to_red_t( &CONTROLLER::move_to_red, this );	
// 	boost::thread avoid_t( &CONTROLLER::avoid, this );
// 	ros::spin();
// }

// int main (int argc, char** argv) {
// 	ros::init(argc, argv, "controller");
// 	CONTROLLER ctrl;
// 	ctrl.run();
// 	return 0;
// }
