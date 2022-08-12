/**
 * @file <src/modules/AutoMoDeConditionColor.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeConditionProbColor.h"

namespace argos
{

    /****************************************/
    /****************************************/

    AutoMoDeConditionProbColor::AutoMoDeConditionProbColor()
    {
        m_strLabel = "ProbColor";
    }

    /****************************************/
    /****************************************/

    AutoMoDeConditionProbColor::~AutoMoDeConditionProbColor() {}

    /****************************************/
    /****************************************/

    AutoMoDeConditionProbColor::AutoMoDeConditionProbColor(AutoMoDeConditionProbColor *pc_condition)
    {
        m_strLabel = pc_condition->GetLabel();
        m_unIndex = pc_condition->GetIndex();
        m_unIdentifier = pc_condition->GetIndex();
        m_unFromBehaviourIndex = pc_condition->GetOrigin();
        m_unToBehaviourIndex = pc_condition->GetExtremity();
        m_mapParameters = pc_condition->GetParameters();
        Init();
    }

    /****************************************/
    /****************************************/

    void AutoMoDeConditionProbColor::Init()
    {
        std::map<std::string, Real>::iterator it = m_mapParameters.find("l");
        if (it != m_mapParameters.end())
        {
            m_cColorParameter = GetColorParameter(it->second);
        }
        else
        {
            LOGERR << "[FATAL] Missing parameter for the following condition:" << m_strLabel << std::endl;
            THROW_ARGOSEXCEPTION("Missing Parameter");
        }
        it = m_mapParameters.find("p");
        if (it != m_mapParameters.end())
        {
            m_fProbability = it->second;
        }
        else
        {
            LOGERR << "[FATAL] Missing parameter for the following condition:" << m_strLabel << std::endl;
            THROW_ARGOSEXCEPTION("Missing Parameter");
        }
    }

    /****************************************/
    /****************************************/

    AutoMoDeConditionProbColor *AutoMoDeConditionProbColor::Clone()
    {
        return new AutoMoDeConditionProbColor(this);
    }

    /****************************************/
    /****************************************/

    bool AutoMoDeConditionProbColor::Verify()
    {
        CColor ptColorPerceived = m_pcRobotDAO->GetGroundReading();
        // saturate the color
        // CColor ptSaturatedColor = Saturate(ptColorPerceived);
        // get closest label
        CColor ptClosestLabel = GetClosestLabel(ptColorPerceived);
        // std::cout << "Color perceived: " << ptColorPerceived << std::endl;
        // std::cout << "Closest label: " << ptClosestLabel << std::endl;
        if (ptClosestLabel == m_cColorParameter)
        {
            return EvaluateBernoulliProbability(m_fProbability);
        }
        return false;
    }

    /****************************************/
    /****************************************/

    void AutoMoDeConditionProbColor::Reset()
    {
        Init();
    }

}