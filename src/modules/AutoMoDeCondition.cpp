/**
 * @file <src/modules/AutoMoDeCondition.cpp>
 *
 * @author Antoine Ligot - <aligot@ulb.ac.be>
 *
 * @package ARGoS3-AutoMoDe
 *
 * @license MIT License
 */

#include "AutoMoDeCondition.h"
#include <limits>

namespace argos
{

	/****************************************/
	/****************************************/

	const std::string AutoMoDeCondition::GetDOTDescription()
	{
		std::stringstream ss;
		ss << m_strLabel;
		if (!m_mapParameters.empty())
		{
			std::map<std::string, Real>::iterator it;
			for (it = m_mapParameters.begin(); it != m_mapParameters.end(); it++)
			{
				ss << "\\n"
				   << it->first << "=" << it->second;
			}
		}
		return ss.str();
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::AddParameter(const std::string &str_identifier, const Real &f_value)
	{
		m_mapParameters.insert(std::pair<std::string, Real>(str_identifier, f_value));
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetOrigin() const
	{
		return m_unFromBehaviourIndex;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetExtremity() const
	{
		return m_unToBehaviourIndex;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetOrigin(const UInt32 &un_from)
	{
		m_unFromBehaviourIndex = un_from;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetExtremity(const UInt32 &un_to)
	{
		m_unToBehaviourIndex = un_to;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetOriginAndExtremity(const UInt32 &un_from, const UInt32 &un_to)
	{
		m_unFromBehaviourIndex = un_from;
		m_unToBehaviourIndex = un_to;
	}

	/****************************************/
	/****************************************/

	const std::string &AutoMoDeCondition::GetLabel() const
	{
		return m_strLabel;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetIndex(const UInt32 &un_index)
	{
		m_unIndex = un_index;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetIndex() const
	{
		return m_unIndex;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetIdentifier(const UInt32 &un_id)
	{
		m_unIdentifier = un_id;
	}

	/****************************************/
	/****************************************/

	const UInt32 &AutoMoDeCondition::GetIdentifier() const
	{
		return m_unIdentifier;
	}

	/****************************************/
	/****************************************/

	std::map<std::string, Real> AutoMoDeCondition::GetParameters() const
	{
		return m_mapParameters;
	}

	/****************************************/
	/****************************************/

	void AutoMoDeCondition::SetRobotDAO(RVRDAO *pc_robot_dao)
	{
		m_pcRobotDAO = pc_robot_dao;
	}

	/****************************************/
	/****************************************/

	bool AutoMoDeCondition::EvaluateBernoulliProbability(const Real &f_probability) const
	{
		return m_pcRobotDAO->GetRandomNumberGenerator()->Bernoulli(f_probability);
	}

	/****************************************/
	/****************************************/

	// Return the color parameter
	CColor AutoMoDeCondition::GetColorParameter(const UInt32 &un_value)
	{
		CColor cColorParameter;
		switch (un_value)
		{
		case 0:
			cColorParameter = CColor::GREEN;
			break;
		case 1:
			// blue physical patches
			cColorParameter = CColor::BLUE;
			break;
		case 2:
			// red physical patches
			cColorParameter = CColor::RED;
			break;
		case 3:
			// yellow physical patches
			cColorParameter = CColor::YELLOW;
			break;
		default:
			cColorParameter = CColor::BLACK;
		}
		return cColorParameter;
	}

	CColor AutoMoDeCondition::Saturate(CColor pc_color)
	{
		CColor fSaturatedColor = CColor();
		UInt8 cMaxChannelValue = pc_color.GetRed();
		if (pc_color.GetGreen() > cMaxChannelValue)
		{
			cMaxChannelValue = pc_color.GetGreen();
		}
		if (pc_color.GetBlue() > cMaxChannelValue)
		{
			cMaxChannelValue = pc_color.GetBlue();
		}
		if (cMaxChannelValue <= 10)
		{
			return CColor::BLACK;
		}
		Real fFactor = 255.0 / cMaxChannelValue;
		fSaturatedColor.SetRed(UInt8(pc_color.GetRed() * fFactor));
		fSaturatedColor.SetGreen(UInt8(pc_color.GetGreen() * fFactor));
		fSaturatedColor.SetBlue(UInt8(pc_color.GetBlue() * fFactor));
		if (fSaturatedColor.GetRed() > 255)
		{
			fSaturatedColor.SetRed(255);
		}
		if (fSaturatedColor.GetGreen() > 255)
		{
			fSaturatedColor.SetGreen(255);
		}
		if (fSaturatedColor.GetBlue() > 255)
		{
			fSaturatedColor.SetBlue(255);
		}
		return fSaturatedColor;
	}

	/****************************************/
	/****************************************/

	CColor AutoMoDeCondition::GetClosestLabel(CColor pc_color)
	{
		CColor cClosestLabel = CColor::WHITE;
		Real fMinDistance = std::numeric_limits<Real>::max();
		Real f_h, f_s, f_v, f_h_current, f_s_current, f_v_current;
		// store perceived color in f_h, f_s, f_v
		RGBtoHSV(pc_color, f_h, f_s, f_v);
		// std::cout << "Hue of perceived color : " << f_h << std::endl;
		for (UInt32 i = 0; i < 4; i++)
		{
			CColor cLabel = GetColorParameter(i);
			RGBtoHSV(cLabel, f_h_current, f_s_current, f_v_current);
			// std::cout << "Color " << cLabel << " | ";
			// std::cout << "Hue " << f_h_current << std::endl;
			Real fDistance = Abs(f_h - f_h_current);
			// std::cout << "Distance :" << fDistance << std::endl;
			if (fDistance < fMinDistance && fDistance < 15 && Abs(f_s - f_s_current) < 0.2)
			{
				fMinDistance = fDistance;
				cClosestLabel = cLabel;
			}
		}
		return cClosestLabel;
	}

	void AutoMoDeCondition::RGBtoHSV(const CColor &c_color, Real &f_h, Real &f_s, Real &f_v)
	{
		// convert RGB to HSV
		Real f_r = c_color.GetRed() / 255.0;
		Real f_g = c_color.GetGreen() / 255.0;
		Real f_b = c_color.GetBlue() / 255.0;
		Real f_min = f_r;
		if (f_g < f_min)
		{
			f_min = f_g;
		}
		if (f_b < f_min)
		{
			f_min = f_b;
		}
		Real f_max = f_r;
		if (f_g > f_max)
		{
			f_max = f_g;
		}
		if (f_b > f_max)
		{
			f_max = f_b;
		}
		Real f_delta = f_max - f_min;
		f_v = f_max;
		if (f_max != 0)
		{
			f_s = f_delta / f_max;
		}
		else
		{
			f_s = 0;
			f_h = 0;
			f_v = 0;
			return;
		}
		if (f_delta == 0)
		{
			f_h = 0;
		}
		else if (f_r == f_max)
		{
			f_h = (f_g - f_b) / f_delta;
		}
		else if (f_g == f_max)
		{
			f_h = 2 + (f_b - f_r) / f_delta;
		}
		else
		{
			f_h = 4 + (f_r - f_g) / f_delta;
		}
		f_h *= 60;
		if (f_h < 0)
		{
			f_h += 360;
		}
	}

}
