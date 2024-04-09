/*
 * @file <src/core/AutoMoDeController.h>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#ifndef AUTOMODE_CONTROLLER_H
#define AUTOMODE_CONTROLLER_H

#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/control_interface/ci_controller.h>

#include <argos3/demiurge/rvr-dao/RVRDAO.h>
#include <argos3/demiurge/rvr-dao/ReferenceModel1Dot2.h>

#include "./AutoMoDeFiniteStateMachine.h"
#include "./AutoMoDeFsmBuilder.h"

#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_wheels_actuator.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_proximity_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_lidar_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_light_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_ground_color_sensor.h>
#include <argos3/plugins/robots/rvr/control_interface/ci_rvr_colored_blob_omnidirectional_camera_sensor.h>

/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

/* ROS dependencies */
#include "ros/ros.h"
#include <tri_msgs/Distance.h>
#include <tri_msgs/Distances.h>

// Include random number generator
#include <argos3/core/utility/math/rng.h>

// Define a type for the pair of floats
typedef std::pair<float, float> DistanceFactorPair;

typedef std::vector<DistanceFactorPair> DistanceTable;

// Define a type for the distance matrix
typedef std::vector<std::vector<DistanceFactorPair>> DistanceMatrix;

namespace argos
{
	class AutoMoDeController : public CCI_Controller
	{
	public:
		/*
		 * Class constructor.
		 */
		AutoMoDeController();

		/*
		 * Class desctructor.
		 */
		virtual ~AutoMoDeController();

		/*
		 * Controller initializer.
		 */
		virtual void Init(TConfigurationNode &t_node);

		/*
		 * Core of the controller.
		 */
		virtual void ControlStep();

		/*
		 *
		 */
		virtual void Reset();

		/*
		 *
		 */
		virtual void Destroy();

        /*
         * ROS related methods.
         */
        virtual void InitROS();

        // TODO: allow for message of ROS to come back (with RaB information)
        // static void CallbackROS(const morpho_msgs::RangeAndBearing::ConstPtr& msg);

        virtual void ControlStepROS();

		/*
		 * Setter for the AutoMoDeFiniteStateMachine.
		 */
		void SetFiniteStateMachine(AutoMoDeFiniteStateMachine *pc_fine_state_machine);

		void SetHistoryFlag(bool b_history_flag);

	private:
		/*
		 * Function that contains all actuations required at the start of an experiment or during the entire experiment.
		 * Example of what you might add in the future: display LED colors, start omnidirectional camera, etc.
		 * This function needs to be called by Reset() in order for the experiment to properly restart.
		 */
		void InitializeActuation();

		/*
		 * Pointer to the finite state machine object that represents the behaviour
		 * of the robot.
		 */
		AutoMoDeFiniteStateMachine *m_pcFiniteStateMachine;

		/*
		 * Pointer to the object representing the state of the robot. This object is
		 * shared with the finite state object AutoMoDeFiniteStateMachine.
		 */
		RVRDAO *m_pcRobotState;

		/*
		 * Time step variable.
		 */
		UInt32 m_unTimeStep;

		/*
		 * Integer part of the robot identifier.
		 */
		UInt32 m_unRobotID;

		/*
		 * String that contains the configuration of the finite state machine.
		 */
		std::string m_strFsmConfiguration;

		/*
		 * Flag that tells whether an history is maintained or not.
		 */
		bool m_bMaintainHistory;

		/*
		 * Flag that tells whether an URL containing a DOT description of the
		 * finite state machine is to be displayed or not.
		 */
		bool m_bPrintReadableFsm;

		/*
		 * Flag that tells whether we are using the real robot or not.
		 */
		bool m_bRealRobot;

		/*
		 * The path to where the history shall be stored.
		 */
		std::string m_strHistoryFolder;

		/*
		 * Pointer to the object in charge of creating the AutoMoDeFiniteStateMachine.
		 */
		AutoMoDeFsmBuilder *m_pcFsmBuilder;

		/*
		 * Pointer to the robot wheels actuator.
		 */
		CCI_RVRWheelsActuator *m_pcWheelsActuator;

        /*
         * Pointer to the robot range-and-bearing actuator.
         */
        CCI_RVRRangeAndBearingActuator* m_pcRabActuator;

		/*
		 * Pointer to the robot proximity sensor.
		 */
		CCI_RVRProximitySensor *m_pcProximitySensor;

		/*
		 * Pointer to the robot lidar sensor.
		 */
		CCI_RVRLidarSensor *m_pcLidarSensor;

		/*
		 * Pointer to the robot light sensor.
		 */
		CCI_RVRLightSensor *m_pcLightSensor;

		/*
		 * Pointer to the robot ground sensor.
		 */
		CCI_RVRGroundColorSensor *m_pcGroundSensor;

		/*
		 * Pointer to the robot omnidirectional camera sensor.
		 */
		CCI_RVRColoredBlobOmnidirectionalCameraSensor *m_pcOmnidirectionalCameraSensor;

        // TODO: might want to put the code specific to UWB simulation in the DAO instead of here
//        /*
//         * Pointer to the robot range-and-bearing sensor.
//         */
//        CCI_RVRRangeAndBearingSensor* m_pcRabSensor;

        /* Pointer to the range and bearing sensor and actuator */
        CCI_RangeAndBearingSensor* m_pcRangeAndBearingSensor; // Receive messages
        CCI_RangeAndBearingActuator* m_pcRangeAndBearingActuator; // Send messages

        UInt16 m_unBandWidth; // allowed bandwidth for the range and bearing communication

        DistanceMatrix m_distanceMatrix; // distance matrix for the range and bearing communication
        DistanceTable m_distanceTable; // distance matrix for the range and bearing communication
        int m_nRobots;

        /* ROS Publisher */
        ros::Publisher m_distancePublisher;
        ros::Publisher m_distancesPublisher;
//        ros::Subscriber m_directionSubscriber;

        tri_msgs::Distance m_distanceMessage;
        tri_msgs::Distances m_distancesMessage;
//        morpho_msgs::Angle m_directionMessage;

		bool m_bFiniteStateMachineGiven;
	};
}

#endif
