/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "realredlightsensor.h"
#include "realbluelightsensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"


/******************************************************************************/
/******************************************************************************/

extern gsl_rng* rng;
extern long int rngSeed;
/******************************************************************************/
/******************************************************************************/

using namespace std;
/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS	6

#define AVOID_PRIORITY 		0
#define RELOAD_PRIORITY 	1
#define SWITCH_PRIORITY		2
#define DRY_PRIORITY		4
#define FORAGE_PRIORITY		3
#define NAVIGATE_PRIORITY 5

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.3
/* Threshold to define the battery discharged */
#define BATTERY_THRESHOLD 0.5//vamos a utilizar una variable nuestra para optimizar carga de bateria
/* Threshold to reduce the speed of the robot */
#define NAVIGATE_LIGHT_THRESHOLD 0.9

#define ERROR_DIRECTION 0.05


#define SPEED 500


/******************************************************************************/
/******************************************************************************/
CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	m_nWriteToFile = n_write_to_file;
	
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor (SENSOR_REAL_RED_LIGHT);
	/* SetBle light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor (SENSOR_REAL_BLUE_LIGHT);

	
	/* Initilize Variables */
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;
	umbralBatt=0.3;
  fBattToForageInhibitor = 1.0;
  fBathroomToForageInhibitor = 1.0;
  fDryToForageInhibitor= 1.0;
  fBathroomToDryInhibitor=1.0;
  fBattToDry=1.0;
  fBattToBathroom=1.0;

	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

	if (m_nWriteToFile ) 
	{
	/* INIT: WRITE TO FILES */
	/* Write robot position and orientation */
		FILE* filePosition = fopen("outputFiles/robotPosition", "a");
		//fprintf(filePosition,"%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
		fprintf(filePosition, "%2.4f %2.4f\n",  m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y);
		fclose(filePosition);

		/* Write robot wheels speed */
		FILE* fileWheels = fopen("outputFiles/robotWheels", "a");
		fprintf(fileWheels,"%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileWheels);
		/* END WRITE TO FILES */
	}

}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Release Inhibitors */
	fBattToForageInhibitor = 1.0;
	fBathroomToForageInhibitor = 1.0;
	fDryToForageInhibitor= 1.0;
	fBathroomToDryInhibitor= 1.0;
	fBattToDry=1.0;
  	fBattToBathroom=1.0;
	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);
	
	ObstacleAvoidance ( AVOID_PRIORITY );
	GoLoad ( RELOAD_PRIORITY );
	SwitchBathroom ( SWITCH_PRIORITY);
	DryFloor (DRY_PRIORITY);
	Forage ( FORAGE_PRIORITY );
	Navigate ( NAVIGATE_PRIORITY );
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator ( void )
{
	/* Create counter for behaviors */
        int       nBehavior;
  /* Create angle of movement */
  double    fAngle = 0.0;
  /* Create vector of movement */
  dVector2  vAngle;
  vAngle.x = 0.0;
  vAngle.y = 0.0;

  /* For every Behavior */
        for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
        {
    /* If behavior is active */
                if ( m_fActivationTable[nBehavior][2] == 1.0 )
                {
      /* DEBUG */
      printf("Behavior %d: %2f\n", nBehavior, m_fActivationTable[nBehavior][0]);
      /* DEBUG */
      vAngle.x += m_fActivationTable[nBehavior][1] * cos(m_fActivationTable[nBehavior][0]);
      vAngle.y += m_fActivationTable[nBehavior][1] * sin(m_fActivationTable[nBehavior][0]);
                }
        }

  /* Calc angle of movement */
  fAngle = atan2(vAngle.y, vAngle.x);
  /* DEBUG */
  printf("fAngle: %2f\n", fAngle);
  printf("\n");
  /* DEBUG */

  if (fAngle > 0)
  {
    m_fLeftSpeed = SPEED*(1 - fmin(fAngle, ERROR_DIRECTION)/ERROR_DIRECTION);
    m_fRightSpeed = SPEED;
  }
  else
  {
    m_fLeftSpeed = SPEED;
    m_fRightSpeed = SPEED*(1 - fmin(-fAngle, ERROR_DIRECTION)/ERROR_DIRECTION);
  }


// 	int nBehavior;
//   double fAngle = 0.0;

//   int nActiveBehaviors = 0;
//   /* For every Behavior Activated, sum angles */
// 	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
// 	{
// 		if ( m_fActivationTable[nBehavior][2] == 1.0 )
// 		{
//       fAngle += m_fActivationTable[nBehavior][0];
//       nActiveBehaviors++;
// 		}
// 	}
//   fAngle /= (double) nActiveBehaviors;
	
//   /* Normalize fAngle */
//   while ( fAngle > M_PI ) fAngle -= 2 * M_PI;
// 	while ( fAngle < -M_PI ) fAngle += 2 * M_PI;
 
//   /* Based on the angle, calc wheels movements */
//   double fCLinear = 1.0;
//   double fCAngular = 1.0;
//   double fC1 = SPEED / M_PI;

//   /* Calc Linear Speed */
//   double fVLinear = SPEED * fCLinear * ( cos ( fAngle / 2) );

//   /*Calc Angular Speed */
//   double fVAngular = fAngle;

//   m_fLeftSpeed  = fVLinear - fC1 * fVAngular;
//   m_fRightSpeed = fVLinear + fC1 * fVAngular;
	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ObstacleAvoidance ( unsigned int un_priority )
{
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);

	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += prox[i] * cos ( proxDirections[i] );
		vRepelent.y += prox[i] * sin ( proxDirections[i] );

		if ( prox[i] > fMaxProx )
			fMaxProx = prox[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxProx;

	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_GREEN);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
	}
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/avoidOutput", "a");
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fprintf(fileOutput, "%2.4f %2.4f\n",m_fTime,m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
	
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Navigate ( unsigned int un_priority )
{
  /* Direction Angle 0.0 and always active. We set its vector intensity to 0.5 if used */
	m_fActivationTable[un_priority][0] = 0.0;
	m_fActivationTable[un_priority][1] = 0.5;
	m_fActivationTable[un_priority][2] = 1.0;

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		//fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fprintf(fileOutput, "%2.4f %2.4f\n",m_fTime,m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}
		
/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoLoad ( unsigned int un_priority )
{
	/* Leer Battery Sensores */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);

	double fMaxLight = 0.0;
	const double* lightDirections = m_seLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
  /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxLight;

	/* If battery below a BATTERY_THRESHOLD */
	if ( battery[0] < umbralBatt )
	{
    /* Inibit Forage, Bathroom and Dry */
		fBattToForageInhibitor = 0.0;
		fBattToDry=0.0;
  		fBattToBathroom=0.0;
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_RED);
		/* Mark behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
		/*ponemos nuevo threshold de bateria, para cargarla más*/
		umbralBatt=0.9;
	}	
	if(battery[0]>=umbralBatt){
			umbralBatt=0.3;
		}

	printf("Bateria: %2.4f\n",battery[0]);
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		//fprintf(fileOutput,"%2.4f %2.4f\n",m_fTime, battery[0]);
		fprintf(fileOutput, "%2.4f %2.4f\n",m_fTime,m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SwitchBathroom ( unsigned int un_priority ){
	/*Leemos sensores de luz roja*/
	double* lightRed = m_seRedLight->GetSensorReading(m_pcEpuck);

	double fMaxLight = 0.0;
	double totalLight = 0;
	const double* lightRedDirections = m_seRedLight->GetSensorDirections();

/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += lightRed[i] * cos ( lightRedDirections[i] );
		vRepelent.y += lightRed[i] * sin ( lightRedDirections[i] );

		if ( lightRed[i] > fMaxLight )
			fMaxLight = lightRed[i];
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);

	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;  //theta
	m_fActivationTable[un_priority][1] = fMaxLight + fMaxLight/4;  //rho

	totalLight= lightRed[0] + lightRed[7];

	if ( totalLight >= 1.36)
	{
		m_seRedLight->SwitchNearestLight(0);
	}
	if(fBattToBathroom==1.0){
		for(int i=0; i< m_seProx->GetNumberOfInputs(); i++){
			if(lightRed[i]!=0.0){
			//Activamos inhibitor
			fBathroomToForageInhibitor = 0.0;
			fBathroomToDryInhibitor= 0.0;
			//Leds a blanco
			m_pcEpuck->SetAllColoredLeds( LED_COLOR_YELLOW);
			//Activamos flag
			m_fActivationTable[un_priority][2] = 1.0;
			}
		}
	}
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/bathroomOutput", "a");
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		//fprintf(fileOutput,"%2.4f %2.4f\n",m_fTime, battery[0]);
		fprintf(fileOutput, "%2.4f %2.4f\n",m_fTime,m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
	
}
/******************************************************************************/
/******************************************************************************/
void CIri1Controller::DryFloor (unsigned int un_priority){
	/*Leemos sensores luz azul*/
	double* lightBlue = m_seBlueLight->GetSensorReading(m_pcEpuck);
	double totalLight = lightBlue[5];

	/*Cogemos código del entregable de Braitenberg: Codigo de Diego*/
	double tmp[2];
	tmp[0]=lightBlue[0]+lightBlue[1]+lightBlue[2];
	tmp[1]=lightBlue[7]+lightBlue[6]+lightBlue[5];
for(int i=0; i< m_seProx->GetNumberOfInputs(); i++){
	if(lightBlue[i]!=0.0){
		if((lightBlue[2]+lightBlue[3]>lightBlue[0]+lightBlue[1])&&(lightBlue[2]!=0)){
			tmp[0]=lightBlue[2]-0.2;
		}
		if((lightBlue[5]+lightBlue[4]>lightBlue[7]+lightBlue[6])&&(lightBlue[5]!=0)){
			tmp[0]=lightBlue[2]-0.2;
		}
		if(tmp[0]==0){
			tmp[0]=(tmp[1]/2)+0.5;
		}else if(tmp[1]==0){
			tmp[1]=(tmp[0]/2)+0.7;
		}
		m_fActivationTable[un_priority][0] = tmp[0];
	 	m_fActivationTable[un_priority][1] = tmp[1];
		if((fBathroomToDryInhibitor * fBattToDry )==1.0){
		m_fActivationTable[un_priority][2] = 1.0;
		fDryToForageInhibitor=0.0;
		if(totalLight>=0.9){
			m_seBlueLight->SwitchNearestLight(0);
		}
		
	}
}

}

	// tmp[0]= lightBlue[0]+lightBlue[1]+lightBlue[2]+lightBlue[3];
	// tmp[1]= lightBlue[7]+lightBlue[6]+lightBlue[5]+lightBlue[4];
	
	// double f_threshold = 0.6;

	// if (tmp[0] > f_threshold){
	// 	tmp[0] = 2 * f_threshold - tmp[0];
		 
	// }

	// if (tmp[1] > f_threshold){
	// 	tmp[1] = 2 * f_threshold - tmp[1];
		 
	// }
	// // m_fLeftSpeed= SPEED*(0.6 + (tmp[1] ));
	// // m_fRightSpeed= SPEED *(0.65 + (tmp[0] ));
	// for(int i=0; i< m_seProx->GetNumberOfInputs(); i++){
	// 	if(lightBlue[i]!=0.0){
	// 		m_fActivationTable[un_priority][0] = SPEED*(0.3 + (tmp[1] ));
	// 		m_fActivationTable[un_priority][1] = SPEED *(0.65 + (tmp[0] ));
	// 		m_fActivationTable[un_priority][2] = 1.0;
	// 	}
	// }

	// m_acWheels->SetOutput(0, 0.6 + (tmp[1] ));
  	// m_acWheels->SetOutput(1, 0.65 + (tmp[0] ));
	// if((tmp[0]>f_threshold )||(tmp[1]>f_threshold )){
	// m_fActivationTable[un_priority][0] = SPEED*(0.6 + (tmp[1] ));
	// m_fActivationTable[un_priority][1] = SPEED *(0.65 + (tmp[0] ));
	// m_fActivationTable[un_priority][2] = 1.0;
	// }
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/DryOutput", "a");
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		//fprintf(fileOutput,"%2.4f %2.4f\n",m_fTime, battery[0]);
		fprintf(fileOutput, "%2.4f %2.4f\n",m_fTime,m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}


}
/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Forage ( unsigned int un_priority )
{
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	
	double fMaxLight = 0.0;
	const double* lightDirections = m_seLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	
  /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = 1 - fMaxLight;
  
  /* If with a virtual puck */
	if ( ( groundMemory[0] * fBattToForageInhibitor * fBathroomToForageInhibitor * fDryToForageInhibitor) == 1.0 )
	{
		/* Set Leds to BLUE */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLUE);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
	/*Activamos Inhibitor para DRY*/
	
		
		/* Go oposite to the light */
		//if ( ( light[3] * light[4] == 0.0 ) )
		//{
			//m_fActivationTable[un_priority][2] = 1.0;

			//double lightLeft 	= light[0] + light[1] + light[2] + light[3];
			//double lightRight = light[4] + light[5] + light[6] + light[7];

			//if ( lightLeft > lightRight )
			//{
				//m_fActivationTable[un_priority][0] = SPEED;
				//m_fActivationTable[un_priority][1] = -SPEED;
			//}
			//else
			//{
				//m_fActivationTable[un_priority][0] = -SPEED;
				//m_fActivationTable[un_priority][1] = SPEED;
			//}
		//}
	}
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/forageOutput", "a");
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, fBattToForageInhibitor, groundMemory[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		//fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fprintf(fileOutput, "%2.4f %2.4f\n",m_fTime,m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

