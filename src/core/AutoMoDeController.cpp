/*
 * @file <src/core/AutoMoDeController.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeController.h"

namespace argos
{

	/****************************************/
	/****************************************/

	AutoMoDeController::AutoMoDeController()
	{
		m_pcRobotState = new ReferenceModel1Dot2();
		m_unTimeStep = 0;
		m_strFsmConfiguration = "";
		m_bMaintainHistory = false;
		m_bPrintReadableFsm = false;
		m_strHistoryFolder = "./";
		m_bFiniteStateMachineGiven = false;
		m_bRealRobot = false;
        m_nRobots = 10;
        m_unBandWidth = 100;
	}

	/****************************************/
	/****************************************/

	AutoMoDeController::~AutoMoDeController()
	{
		delete m_pcRobotState;
		if (m_strFsmConfiguration.compare("") != 0)
		{
			delete m_pcFsmBuilder;
		}
	}

    /****************************************/
/****************************************/

//    void CFootBotWalk::InitROS() {
//        //get e-puck ID
//        std::stringstream name;
//        name.str("");
//        name << GetId(); // fbX
//
//        //init ROS
//        if (!ros::isInitialized()) {
//            char **argv = NULL;
//            int argc = 0;
//            ros::init(argc, argv, name.str());
//        }
//
//        // ROS access node
//        ros::NodeHandle pub_node;
//        ros::NodeHandle self_pub_node;
////        ros::NodeHandle sub_node;
//
//        std::stringstream publisherName;
//        std::stringstream selfPublisherName;
////        std::stringstream subscriberName;
//
//        publisherName << name.str() << "/distances";
//        selfPublisherName << name.str() << "/distance";
////        subscriberName << name.str() << "/range_and_bearing";
//
//        // Register the publisher to the ROS master
//        m_distancePublisher = self_pub_node.advertise<tri_msgs::Distance>(selfPublisherName.str(), 10);
//        m_distancesPublisher = pub_node.advertise<tri_msgs::Distances>(publisherName.str(), 10);
////        m_directionSubscriber = sub_node.subscribe(subscriberName.str(), 10, CallbackROS);
//    }
//
////    void CFootBotWalk::CallbackROS(const morpho_msgs::RangeAndBearing::ConstPtr& msg) {
////        m_go = msg->go;
////    }
//
//    void CFootBotWalk::ControlStepROS() {
//        if (ros::ok()) {
//            // Publish the message
//            if (m_distancesMessage.robot_id != 0) {
//                m_distancesPublisher.publish(m_distancesMessage);
//
//                // Clean message for next iteration
//                m_distancesMessage.robot_id = 0;
//                m_distancesMessage.ranges.clear();
//            }
//            if (m_distanceMessage.other_robot_id != 0) {
//                m_distancePublisher.publish(m_distanceMessage);
//
//                // Clean the message for next iteration
//                m_distanceMessage.other_robot_id = 0;
//            }
//
//            //update ROS status
//            ros::spinOnce();
//        }
//    }

	/****************************************/
	/****************************************/

	void AutoMoDeController::Init(TConfigurationNode &t_node)
	{
		// Parsing parameters
		Real ptVelocity = m_pcRobotState->GetMaxVelocity();
		try
		{
			GetNodeAttributeOrDefault(t_node, "fsm-config", m_strFsmConfiguration, m_strFsmConfiguration);
			GetNodeAttributeOrDefault(t_node, "history", m_bMaintainHistory, m_bMaintainHistory);
			GetNodeAttributeOrDefault(t_node, "hist-folder", m_strHistoryFolder, m_strHistoryFolder);
			GetNodeAttributeOrDefault(t_node, "readable", m_bPrintReadableFsm, m_bPrintReadableFsm);
			GetNodeAttributeOrDefault(t_node, "real-robot", m_bRealRobot, m_bRealRobot);
			GetNodeAttributeOrDefault(t_node, "velocity", ptVelocity, ptVelocity);

            // RaB related configuration
            GetNodeAttributeOrDefault(t_node, "comm_size", m_unBandWidth, m_unBandWidth);
            GetNodeAttributeOrDefault(t_node, "num_robots", m_nRobots, m_nRobots);

            // Fill the distance table with ones
            m_distanceTable.resize(m_nRobots, DistanceFactorPair(0, 1));
            // TODO: put ROS
//            InitROS();
		}
		catch (CARGoSException &ex)
		{
			THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
		}
		m_pcRobotState->SetMaxVelocity(ptVelocity);
		m_unRobotID = atoi(GetId().substr(5, 6).c_str());
		m_pcRobotState->SetRobotIdentifier(m_unRobotID);

		/*
		 * If a FSM configuration is given as parameter of the experiment file, create a FSM from it
		 */
		if (m_strFsmConfiguration.compare("") != 0 && !m_bFiniteStateMachineGiven)
		{
			m_pcFsmBuilder = new AutoMoDeFsmBuilder();
			SetFiniteStateMachine(m_pcFsmBuilder->BuildFiniteStateMachine(m_strFsmConfiguration));
			if (m_bMaintainHistory)
			{
				m_pcFiniteStateMachine->SetHistoryFolder(m_strHistoryFolder);
				m_pcFiniteStateMachine->MaintainHistory();
			}
			if (m_bPrintReadableFsm)
			{
				std::cout << "Finite State Machine description: " << std::endl;
				std::cout << m_pcFiniteStateMachine->GetReadableFormat() << std::endl;
			}
		}
		else
		{
			LOGERR << "Warning: No finite state machine configuration found in .argos" << std::endl;
		}

		/*
		 *  Initializing sensors and actuators
		 */
		if (m_bRealRobot)
		{
			return;
		}
		try
		{
			m_pcProximitySensor = GetSensor<CCI_RVRProximitySensor>("rvr_proximity");
			m_pcLightSensor = GetSensor<CCI_RVRLightSensor>("rvr_light");
			m_pcGroundSensor = GetSensor<CCI_RVRGroundColorSensor>("rvr_ground");
			m_pcLidarSensor = GetSensor<CCI_RVRLidarSensor>("rvr_lidar");
			m_pcOmnidirectionalCameraSensor = GetSensor<CCI_RVRColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
			m_pcOmnidirectionalCameraSensor->Enable();

            // Add the range and bearing sensor and actuator
            m_pcRangeAndBearingSensor = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
            m_pcRangeAndBearingActuator = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
		}
		catch (CARGoSException ex)
		{
			LOGERR << "Error while initializing a Sensor!\n";
		}

		try
		{
			m_pcWheelsActuator = GetActuator<CCI_RVRWheelsActuator>("rvr_wheels");
            m_pcRabActuator = GetActuator<CCI_RVRRangeAndBearingActuator>("rvr_range_and_bearing");
		}
		catch (CARGoSException ex)
		{
			LOGERR << "Error while initializing an Actuator!\n";
		}

		/*
		 * Starts actuation.
		 */
		InitializeActuation();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::ControlStep()
	{

		/*
		 * 1. Update RobotDAO
		 */
		if (!m_bRealRobot)
		{
			if (m_pcGroundSensor != NULL)
			{
				const CCI_RVRGroundColorSensor::SReading &reading = m_pcGroundSensor->GetReading();
				m_pcRobotState->SetGroundInput(reading);
			}
			if (m_pcLightSensor != NULL)
			{
				const CCI_RVRLightSensor::SReading &reading = m_pcLightSensor->GetReading();
				m_pcRobotState->SetLightInput(reading);
			}
			if (m_pcProximitySensor != NULL)
			{
				const CCI_RVRProximitySensor::TReadings &readings = m_pcProximitySensor->GetReadings();
				m_pcRobotState->SetProximityInput(readings);
			}
			if (m_pcLidarSensor != NULL)
			{
				const CCI_RVRLidarSensor::TReadings &readings = m_pcLidarSensor->GetReadings();
				m_pcRobotState->SetLidarInput(readings);
			}
			if (m_pcOmnidirectionalCameraSensor != NULL)
			{
				const CCI_RVRColoredBlobOmnidirectionalCameraSensor::SReadings &readings = m_pcOmnidirectionalCameraSensor->GetReadings();
				m_pcRobotState->SetOmnidirectionalCameraInput(readings);
			}
            if (m_pcRabSensor != NULL)
            {
                /* Simulate Communication */

                /** Update the Certainty Factor of the measurements */

                for (uint16_t j = 0; j < m_nRobots; ++j) {
                    m_distanceTable[j].second = m_distanceTable[j].second * 0.99f;
                }

                /** Initiator */
                /* Note: only simulating the reception of the acknowledgment message, which is sent by the responder
                 *       after receiving the message from the initiator.
                 *       => receives the message that allows for distance estimation
                 */

                // Get readings from range and bearing sensor
                const CCI_RangeAndBearingSensor::TReadings &tPackets = m_pcRangeAndBearingSensor->GetReadings();

                UInt8 initiator_id;
                UInt8 responder_id;

                if (!tPackets.empty()) {
                    size_t un_SelectedPacket = CRandom::CreateRNG("argos")->Uniform(CRange<UInt32>(0, tPackets.size()));

                    CByteArray data = tPackets[un_SelectedPacket].Data;

                    data >> initiator_id;
                    data >> responder_id;

                    // Save the estimated range (just measured)
                    if (initiator_id != '\0' && responder_id != '\0') {
                        // TODO: could also want to replace by average of previous and current to even out the result
                        m_distanceTable[(int) responder_id - 'A'] = std::make_pair(tPackets[un_SelectedPacket].Range, 1);

                        m_distanceMessage.other_robot_id = (int) responder_id;
                        m_distanceMessage.distance = tPackets[un_SelectedPacket].Range;
                        m_distanceMessage.certainty = 100;
                    }

                    float range;
                    float certainty;

                    tri_msgs::Distance item;

                    m_distancesMessage.robot_id = (int) responder_id;

                    for (int k = 0; k < m_nRobots; ++k) {
                        data >> range;
                        data >> certainty;

                        item.other_robot_id = 'A' + k;
                        item.distance = (float) range;
                        item.certainty = (int) ((float) certainty * 100) ;

                        m_distancesMessage.ranges.push_back(item);
                    }
                }
            }
		}

		/*
		 * 2. Execute step of FSM
		 */
		m_pcFiniteStateMachine->ControlStep();

		/*
		 * 3. Update Actuators
		 */
		if (m_bRealRobot)
		{
			// make sure we publish at each time step in case ROS is used
			m_pcRobotState->SetWheelsVelocity(m_pcRobotState->GetLeftWheelVelocity(), m_pcRobotState->GetRightWheelVelocity());
		}
		else
		{
			if (m_pcWheelsActuator != NULL)
			{
				m_pcWheelsActuator->SetLinearVelocity(m_pcRobotState->GetLeftWheelVelocity(), m_pcRobotState->GetRightWheelVelocity());
			}
            if (m_pcRabActuator != NULL)
            {
                /** Responder */
                /*  Note: only simulating the sending of the acknowledgment message, which is sent by the responder
                 *        after receiving the message from the initiator.
                 *        => sends the message that allows for distance estimation
                 */

                /* Send a message
                 * 1. ID of sender (robot that is supposed to receive the message)
                 * 2. ID of receiver (itself)
                 * 3. Information (distance update)
                 */
                CByteArray cMessage;
                cMessage << (UInt8) 'A';          // ID of sender ('A' is the broadcast ID)
                cMessage << (UInt8) GetId()[10];   // ID of receiver

                for (uint16_t j = 0; j < m_nRobots; ++j) { // 32 bit = 4 bytes => requires 10 * (4 * 2) = 80 bytes of data for 10 robots
                    cMessage << (float) m_distanceTable[j].first;
                    cMessage << (float) m_distanceTable[j].second;
                }

                // Fill to the size of communication allowed (bytes)
                cMessage.Resize(m_unBandWidth, '\0');

                // Send the message
                m_pcRangeAndBearingActuator->SetData(cMessage);
            }
		}

        /*
		 * 4. Update variables and sensors
		 */
        if (m_pcRabSensor != NULL) {
            m_pcRabSensor->ClearPackets();
        }

		m_unTimeStep++;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::Destroy() {}

	/****************************************/
	/****************************************/

	void AutoMoDeController::Reset()
	{
		m_pcFiniteStateMachine->Reset();
		m_pcRobotState->Reset();

        // Fill the distance table with ones
        m_distanceTable.resize(m_nRobots, DistanceFactorPair(0, 1));

		// Restart actuation.
		InitializeActuation();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::SetFiniteStateMachine(AutoMoDeFiniteStateMachine *pc_finite_state_machine)
	{
		m_pcFiniteStateMachine = pc_finite_state_machine;
		m_pcFiniteStateMachine->SetRobotDAO(m_pcRobotState);
		m_pcFiniteStateMachine->Init();
		m_bFiniteStateMachineGiven = true;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::SetHistoryFlag(bool b_history_flag)
	{
		if (b_history_flag)
		{
			m_pcFiniteStateMachine->MaintainHistory();
		}
	}

	/****************************************/
	/****************************************/

	void AutoMoDeController::InitializeActuation()
	{
        /*
		 * Constantly send range-and-bearing messages containing the robot integer identifier.
		 */
        if (m_pcRabActuator != NULL) {
            UInt8 data[4];
            data[0] = m_unRobotID;
            data[1] = 0;
            data[2] = 0;
            data[3] = 0;
            m_pcRabActuator->SetData(data);
        }
	}

	REGISTER_CONTROLLER(AutoMoDeController, "automode_controller");
}
