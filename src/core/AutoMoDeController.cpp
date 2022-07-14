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
		m_pcRobotState = new ReferenceModel1Dot1();
		m_unTimeStep = 0;
		m_strFsmConfiguration = "";
		m_bMaintainHistory = false;
		m_bPrintReadableFsm = false;
		m_strHistoryFolder = "./";
		m_bFiniteStateMachineGiven = false;
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

	void AutoMoDeController::Init(TConfigurationNode &t_node)
	{
		// Parsing parameters
		try
		{
			GetNodeAttributeOrDefault(t_node, "fsm-config", m_strFsmConfiguration, m_strFsmConfiguration);
			GetNodeAttributeOrDefault(t_node, "history", m_bMaintainHistory, m_bMaintainHistory);
			GetNodeAttributeOrDefault(t_node, "hist-folder", m_strHistoryFolder, m_strHistoryFolder);
			GetNodeAttributeOrDefault(t_node, "readable", m_bPrintReadableFsm, m_bPrintReadableFsm);
		}
		catch (CARGoSException &ex)
		{
			THROW_ARGOSEXCEPTION_NESTED("Error parsing <params>", ex);
		}

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
		try
		{
			m_pcProximitySensor = GetSensor<CCI_RVRProximitySensor>("rvr_proximity");
			m_pcLightSensor = GetSensor<CCI_RVRLightSensor>("rvr_light");
			m_pcGroundSensor = GetSensor<CCI_RVRGroundColorSensor>("rvr_ground");
			m_pcLidarSensor = GetSensor<CCI_RVRLidarSensor>("rvr_lidar");
		}
		catch (CARGoSException ex)
		{
			LOGERR << "Error while initializing a Sensor!\n";
		}

		try
		{
			m_pcWheelsActuator = GetActuator<CCI_RVRWheelsActuator>("rvr_wheels");
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

		/*
		 * 2. Execute step of FSM
		 */
		m_pcFiniteStateMachine->ControlStep();

		/*
		 * 3. Update Actuators
		 */
		if (m_pcWheelsActuator != NULL)
		{
			m_pcWheelsActuator->SetLinearVelocity(m_pcRobotState->GetLeftWheelVelocity(), m_pcRobotState->GetRightWheelVelocity());
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
	}

	REGISTER_CONTROLLER(AutoMoDeController, "automode_controller");
}
